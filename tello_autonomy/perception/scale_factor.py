"""
perception/scale_factor.py

Pure math + data-prep for scale-factor computation: bilinear depth
sampling, per-keyframe ratio computation, per-map_id median grouping,
and frame-index/matching helpers for lining up keyframe timestamps
with saved frame images on disk. No torch, no ROS2 - this module never
imports perception.depth.depth_anything_v2 directly; the model
inference function is passed in (dependency injection), so this stays
testable without a GPU/model loaded, and the "only file allowed to
import torch" rule (handoff doc, Section 7) stays enforced by
construction, not just convention.

**Critical invariant, repeated from the handoff doc (Section 8):**
scale factors are NEVER comparable across different map_id values.
Each ORB-SLAM3 map (re)initialization picks its own arbitrary,
unmeasurable baseline for its first two frames' triangulation. Every
function below that touches ratios takes data already filtered to a
single map_id, or filters to one internally - ratios from different
map_ids must never be pooled or averaged together.
"""

import glob
import os
import re

import cv2
import numpy as np

from config import constants


FRAME_FILENAME_RE = re.compile(r"frame_(\d+\.\d+)_(\d+)\.jpg$")


# ---------------------------------------------------------------------------
# Frame indexing / matching (data on already-saved files)
# ---------------------------------------------------------------------------

def build_frame_index(frames_dir=None):
    """
        Maps timestamp (float, seconds) -> filepath for every saved
        frame image, using the timestamp embedded directly in the
        filename (frame_<timestamp>_<frame_id>.jpg - see
        middleware.ros_bridge / message_types.current_timestamp_msg
        for where that filename format comes from).

        frames_dir defaults to constants.SAVE_FRAMES_DIR.
    """
    frames_dir = frames_dir or constants.SAVE_FRAMES_DIR
    index = {}
    for filepath in glob.glob(os.path.join(frames_dir, "frame_*.jpg")):
        filename = os.path.basename(filepath)
        match = FRAME_FILENAME_RE.match(filename)
        if not match:
            continue
        timestamp_str, _frame_id = match.groups()
        index[float(timestamp_str)] = filepath
    return index


def find_matching_frame(target_ts, frame_index, tolerance=None):
    """
        Nearest frame timestamp within tolerance, or None if nothing
        close enough. tolerance defaults to
        constants.KEYFRAME_MATCH_TOLERANCE_SEC (the same tolerance
        middleware.frame_cleanup uses for the equivalent matching
        problem in the other direction).
    """
    tolerance = tolerance if tolerance is not None else constants.KEYFRAME_MATCH_TOLERANCE_SEC

    best_ts, best_diff = None, None
    for ts in frame_index:
        diff = abs(ts - target_ts)
        if best_diff is None or diff < best_diff:
            best_ts, best_diff = ts, diff
    if best_diff is not None and best_diff <= tolerance:
        return frame_index[best_ts]
    return None


# ---------------------------------------------------------------------------
# Pure math
# ---------------------------------------------------------------------------

def sample_depth_bilinear(depth_map, u, v):
    """
        Bilinear sample of depth_map (H, W) at floating-point pixel
        coordinates (u, v). CSV pixel coordinates are sub-pixel
        accurate (from ORB-SLAM3's undistorted keypoints), so
        nearest-pixel lookup would throw away some of that precision -
        bilinear keeps it.
    """
    h, w = depth_map.shape
    u = np.clip(u, 0, w - 1 - 1e-3)
    v = np.clip(v, 0, h - 1 - 1e-3)

    u0, v0 = int(np.floor(u)), int(np.floor(v))
    u1, v1 = u0 + 1, v0 + 1
    du, dv = u - u0, v - v0

    d00 = depth_map[v0, u0]
    d01 = depth_map[v0, u1]
    d10 = depth_map[v1, u0]
    d11 = depth_map[v1, u1]

    return (d00 * (1 - du) * (1 - dv) + d01 * du * (1 - dv)
            + d10 * (1 - du) * dv + d11 * du * dv)


def compute_ratios_for_keyframe(kf_rows, metric_depth_map, min_slam_depth=None):
    """
        kf_rows: iterable of dict-like rows for ONE keyframe (e.g. from
        a pandas DataFrame slice's .to_dict("records")), each with
        'depth_camera_frame', 'pixel_u', 'pixel_v' keys.

        Returns a list of (metric_depth / slam_depth) ratios for that
        single keyframe. Points with near-zero SLAM depth or
        non-positive predicted metric depth are skipped (divide-by-
        near-zero / invalid-prediction guards).
    """
    min_slam_depth = min_slam_depth if min_slam_depth is not None else constants.MIN_SLAM_DEPTH

    ratios = []
    for row in kf_rows:
        slam_depth = row["depth_camera_frame"]
        if slam_depth < min_slam_depth:
            continue
        metric_depth = sample_depth_bilinear(metric_depth_map, row["pixel_u"], row["pixel_v"])
        if metric_depth <= 0:
            continue
        ratios.append(metric_depth / slam_depth)
    return ratios


def compute_scale_factor_from_ratios(ratios_by_keyframe):
    """
        ratios_by_keyframe: list of lists (one list of ratios per
        keyframe), ALL FROM THE SAME map_id - see the module docstring;
        callers must never pass in ratios from more than one map_id
        here.

        Returns (scale_factor, num_points_used):
          - (None, 0) if there are no usable ratios at all.
          - (median_ratio, count) otherwise.
    """
    all_ratios = [r for kf_ratios in ratios_by_keyframe for r in kf_ratios]
    if not all_ratios:
        return None, 0
    return float(np.median(all_ratios)), len(all_ratios)


# ---------------------------------------------------------------------------
# Per-map_id, "recent N keyframes" orchestration
# ---------------------------------------------------------------------------

def compute_scale_factor_for_recent_keyframes(
    keyframe_df,
    map_id,
    frame_index,
    infer_metric_depth_fn,
    recent_count=None,
    frame_match_tolerance=None,
    min_slam_depth=None,
    inference_stride=None,
):
    """
        Computes a scale factor for a single map_id, using only its
        most recent `recent_count` keyframes (by keyframe_id order) -
        NOT every keyframe collected since the map was created. This
        is deliberate: recent keyframes reflect current tracking
        conditions, and re-running inference over every keyframe ever
        seen for a long-lived map_id would only get slower and slower
        over the course of a flight for no accuracy benefit.

        keyframe_df: pandas DataFrame with columns keyframe_id,
            timestamp, map_id, pixel_u, pixel_v, depth_camera_frame
            (the schema written by the C++ node's
            WriteKeyframeDataToFile - see handoff doc, Section 8).
        map_id: which map_id to compute for. Rows are filtered to this
            map_id ONLY.
        frame_index: dict from build_frame_index(), timestamp -> filepath.
        infer_metric_depth_fn: callable(image_bgr) -> (H, W) metric
            depth array in meters. Callers pass
            perception.depth.depth_anything_v2.infer_metric_depth here
            - this module itself never imports torch (see module
            docstring).
        recent_count: defaults to constants.SCALE_FACTOR_RECENT_KEYFRAME_COUNT.
        frame_match_tolerance: defaults to constants.KEYFRAME_MATCH_TOLERANCE_SEC.
        min_slam_depth: defaults to constants.MIN_SLAM_DEPTH.

        Returns a dict:
          {
            "map_id": map_id,
            "scale_factor": float or None,
            "num_points_used": int,
            "num_keyframes_used": int,
            "num_keyframes_skipped": int,
          }
        scale_factor is None if no usable point ratios were collected
        (e.g. no keyframes for this map_id matched a saved frame image
        within tolerance) - callers should treat that as "try again
        next tick", not as an error.
    """
    recent_count = recent_count if recent_count is not None else constants.SCALE_FACTOR_RECENT_KEYFRAME_COUNT
    frame_match_tolerance = (
        frame_match_tolerance if frame_match_tolerance is not None
        else constants.KEYFRAME_MATCH_TOLERANCE_SEC
    )
    min_slam_depth = min_slam_depth if min_slam_depth is not None else constants.MIN_SLAM_DEPTH
    # inference_stride: run Depth Anything on 1 in every N matched keyframes.
    # Adjacent keyframes overlap heavily so skipping them barely hurts accuracy
    # but cuts inference time by 1/stride. Default 2 = half as many model calls.
    inference_stride = inference_stride if inference_stride is not None else constants.DEPTH_INFERENCE_STRIDE

    map_rows = keyframe_df[keyframe_df["map_id"] == map_id]
    keyframe_ids = sorted(map_rows["keyframe_id"].unique())
    recent_keyframe_ids = keyframe_ids[-recent_count:]  # most recent N only, per the design above

    ratios_by_keyframe = []
    keyframes_used = 0
    keyframes_skipped = 0
    matched_count = 0  # count of keyframes that found a saved image

    for kf_id in recent_keyframe_ids:
        kf_rows = map_rows[map_rows["keyframe_id"] == kf_id]
        kf_timestamp = kf_rows["timestamp"].iloc[0]

        image_path = find_matching_frame(kf_timestamp, frame_index, tolerance=frame_match_tolerance)
        if image_path is None:
            keyframes_skipped += 1
            continue

        # --- Frame subsampling: only run expensive model inference on
        # every Nth matched keyframe (stride=1 means all, stride=2 means
        # every other one, etc). Skipped frames still count toward the
        # keyframe budget but don't contribute ratios.
        matched_count += 1
        if (matched_count - 1) % inference_stride != 0:
            keyframes_skipped += 1
            continue

        image_bgr = cv2.imread(image_path)
        if image_bgr is None:
            keyframes_skipped += 1
            continue

        metric_depth_map = infer_metric_depth_fn(image_bgr)
        kf_ratios = compute_ratios_for_keyframe(
            kf_rows.to_dict("records"), metric_depth_map, min_slam_depth
        )

        if kf_ratios:
            ratios_by_keyframe.append(kf_ratios)
            keyframes_used += 1
        else:
            keyframes_skipped += 1

    scale_factor, num_points = compute_scale_factor_from_ratios(ratios_by_keyframe)

    return {
        "map_id": map_id,
        "scale_factor": scale_factor,
        "num_points_used": num_points,
        "num_keyframes_used": keyframes_used,
        "num_keyframes_skipped": keyframes_skipped,
    }
