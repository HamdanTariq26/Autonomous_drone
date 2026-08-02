"""
perception/depth_inference_worker.py

Runs Depth Anything V2 inference in a SEPARATE OS PROCESS (not a
thread) from the rest of this package, so it can be given a lower OS
scheduling priority (via os.nice()) than flight control, ROS2
callbacks, and frame publishing. A Python thread can't achieve this:
threads share their parent process's single OS priority (and the
GIL), so a CPU-bound inference thread can still starve other threads
in the same process regardless of any niceness setting - this is what
was actually causing the drone-control stutter/wonkiness observed
during recompute, not a logic bug in the threading itself.

This module's run_worker() is the entry point for that process. It:
  1. Sets its OWN niceness to constants.DEPTH_WORKER_NICE_VALUE (lower
     OS scheduling priority - it only gets CPU time other, higher-
     priority processes/threads aren't currently using).
  2. Loops forever, pulling one request at a time off request_queue.
  3. For each request, calls perception.scale_factor.
     compute_scale_factor_for_recent_keyframes(), injecting
     perception.depth.depth_anything_v2.infer_metric_depth as the
     model-inference function. The torch import and model loading
     happen ONLY inside this process - the main process (running
     ROS2 callbacks, manual flight control, etc.) never imports torch
     at all.
  4. Puts a result dict back on result_queue.

Requests/results are plain picklable dicts (not DataFrames, not ROS2
messages, not Node references) - keeping the boundary between this
process and the main process to simple data only, since anything tied
to rclpy or a Node cannot cross a process boundary.

IMPORTANT: run_worker() is meant to be used as the `target` of a
multiprocessing.Process created via
multiprocessing.get_context("spawn") - see
perception.scale_factor_manager.ScaleFactorManager.__init__. Do not
call this function directly from the main process.
"""

import os
import queue

from config import constants


def run_worker(request_queue, result_queue):
    """
        Process entry point. Blocks forever (until the shutdown
        sentinel None is received on request_queue), so this must run
        on its own process, never inline.
    """
    try:
        os.nice(constants.DEPTH_WORKER_NICE_VALUE)
    except (AttributeError, OSError) as e:
        # os.nice() can raise if the requested niceness isn't
        # permitted (e.g. certain sandboxed environments). Not fatal -
        # worse case is this process runs at normal priority instead
        # of a lowered one. Never crash the worker over this.
        print(f"[depth_inference_worker] Could not set niceness: {e}")

    # Imported here, inside the worker process's own function body -
    # NOT at module level. This module (depth_inference_worker.py) may
    # be imported by the main process just to reference run_worker
    # itself; that import must stay cheap and torch-free. The heavy
    # imports below only ever execute once this function actually
    # starts running, which only happens inside the spawned process.
    import pandas as pd

    from perception import scale_factor
    from perception.depth import depth_anything_v2
    from perception import camera_intrinsics
    from perception.depth_backprojection import backproject_depth_to_camera_points

    try:
        intrinsics = camera_intrinsics.load_intrinsics()
        print(f"[depth_inference_worker] Loaded camera intrinsics: {intrinsics}")
    except (FileNotFoundError, KeyError) as e:
        intrinsics = None
        print(f"[depth_inference_worker] Could not load camera intrinsics ({e}) - "
              f"proceeding WITHOUT undistortion.")

    print(f"[depth_inference_worker] Started (pid={os.getpid()}), waiting for requests...")

    while True:
        request = request_queue.get()  # blocks until a request arrives

        if request is None:  # shutdown sentinel - see ScaleFactorManager.destroy_node()
            print("[depth_inference_worker] Received shutdown sentinel, exiting.")
            break

        map_id = request["map_id"]
        rows = request["rows"]

        try:
            import cv2

            df = pd.DataFrame(rows)
            df["map_id"] = map_id

            frame_index = scale_factor.build_frame_index(constants.SAVE_FRAMES_DIR)

            # --- Scale factor computation (existing behaviour, unchanged) ---
            result = scale_factor.compute_scale_factor_for_recent_keyframes(
                keyframe_df=df,
                map_id=map_id,
                frame_index=frame_index,
                infer_metric_depth_fn=lambda img: depth_anything_v2.infer_metric_depth(
                    img, intrinsics=intrinsics
                ),
            )
            result["error"] = None

            # --- Dense back-projection: reuse the MOST RECENT matched keyframe's depth map ---
            # We back-project one depth map per result (the most recent keyframe that ran
            # inference) so ScaleFactorManager can publish a dense point cloud without
            # repeating inference. "Most recent" = highest keyframe_id among those that
            # actually produced ratios (inference_stride may have skipped some).
            # This is a best-effort addition: if anything fails we simply omit dense_points_cam
            # so the existing scale-factor path is never broken by this new step.
            result["dense_points_cam"] = None
            result["dense_kf_id"] = None

            if intrinsics is not None:
                map_rows = df[df["map_id"] == map_id]
                keyframe_ids = sorted(map_rows["keyframe_id"].unique())
                recent_count = constants.SCALE_FACTOR_RECENT_KEYFRAME_COUNT
                stride = constants.DEPTH_INFERENCE_STRIDE
                recent_kf_ids = keyframe_ids[-recent_count:]

                matched_count = 0
                best_depth_map = None
                best_kf_id = None

                for kf_id in recent_kf_ids:
                    kf_rows = map_rows[map_rows["keyframe_id"] == kf_id]
                    kf_timestamp = kf_rows["timestamp"].iloc[0]
                    image_path = scale_factor.find_matching_frame(kf_timestamp, frame_index)
                    if image_path is None:
                        continue
                    matched_count += 1
                    if (matched_count - 1) % stride != 0:
                        continue
                    image_bgr = cv2.imread(image_path)
                    if image_bgr is None:
                        continue
                    # This re-runs inference for the last eligible keyframe only.
                    # Since compute_scale_factor_for_recent_keyframes already ran inference
                    # on the same set, the model is warm (cached in the process), so this
                    # extra call does not significantly increase latency.
                    best_depth_map = depth_anything_v2.infer_metric_depth(image_bgr, intrinsics=intrinsics)
                    best_kf_id = int(kf_id)

                if best_depth_map is not None:
                    points_cam = backproject_depth_to_camera_points(
                        depth_m=best_depth_map,
                        intrinsics=intrinsics,
                    )
                    result["dense_points_cam"] = points_cam
                    result["dense_kf_id"] = best_kf_id

        except Exception as e:
            result = {
                "map_id": map_id,
                "scale_factor": None,
                "num_points_used": 0,
                "num_keyframes_used": 0,
                "num_keyframes_skipped": 0,
                "error": str(e),
                "dense_points_cam": None,
                "dense_kf_id": None,
            }

        result_queue.put(result)
