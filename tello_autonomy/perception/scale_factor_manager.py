"""
perception/scale_factor_manager.py

ROS2 node that decides WHEN to (re)compute a scale factor for each
active map_id, and dispatches that work to a separate OS PROCESS to do
it - see perception/depth_inference_worker.py for why a process rather
than a thread (short version: threads share their parent's OS
scheduling priority, so a CPU-bound inference thread can still starve
flight control even on its own thread; a process can be os.nice()'d
lower than everything else, a thread cannot).

CHANGED (this revision): recomputation used to be dispatched onto a
threading.Thread per attempt. That thread still ran inference on CPU
inside THIS process, competing for the same OS-scheduled CPU time as
manual_control's flight-control loop and the ROS2 executor thread -
which is what caused the observed control stutter/lag right after
takeoff, correlated with recompute timing. This revision replaces that
per-attempt thread with ONE long-lived worker PROCESS (started once in
__init__, not per-request - process startup via "spawn" plus model
loading is too slow to redo every 15s) that the model lives inside for
the whole node's lifetime. Recompute requests go out over a
multiprocessing.Queue; results come back over a second one, drained by
a lightweight, non-blocking timer on the ROS2 executor thread.

perception.scale_factor's public interface is UNCHANGED - it's now
called from inside the worker process (see depth_inference_worker.py)
instead of a thread in this process, but its function signatures and
behavior are untouched.

Map_id lifecycle tracking (new map_id detection via keyframe_points
arrival, staleness pruning via TOPIC_MAP_TOPOLOGY_CHANGED +
MAP_DATA_STALE_AFTER_SEC) is UNCHANGED from the previous revision - see
that logic below, only the dispatch mechanism changed.
"""

import queue as queue_module
import time
from collections import OrderedDict

import multiprocessing
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32, String
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from config import constants
from middleware.topic_manager import TopicManager
from perception.depth_inference_worker import run_worker
from perception.pose_transform import (
    transform_camera_points_to_world,
    _quaternion_to_rotation_matrix,
    _rotation_matrix_to_quat,
)


def _parse_map_id_from_frame_id(frame_id):
    """Same encoding used by perception.live_scaler - see that module for details."""
    if not frame_id.startswith(constants.SLAM_MAP_FRAME_ID_PREFIX):
        return None
    suffix = frame_id[len(constants.SLAM_MAP_FRAME_ID_PREFIX):]
    try:
        return int(suffix)
    except ValueError:
        return None


# Explicit "spawn" context - deliberately NOT the platform default
# ("fork" on Linux). Forking a process that has already imported
# torch/numpy/OpenMP is a known source of hangs/deadlocks in PyTorch's
# multiprocessing guidance; "spawn" starts a genuinely fresh
# interpreter in the child instead, which costs a bit of startup time
# but avoids that whole class of problem. See depth_inference_worker.py.
_MP_CONTEXT = multiprocessing.get_context("spawn")


class ScaleFactorManager(Node):
    """
        Caches the latest keyframe_points snapshot per map_id (from
        TOPIC_KEYFRAME_POINTS) and runs a per-map_id periodic recompute
        timer, dispatching the actual work to a long-lived worker
        process. Current best scale factor per map_id is available via
        self.current_scale_factors (a plain dict, map_id -> float), for
        any other node/layer (e.g. perception.live_scaler) to read.
    """

    def __init__(self, node_name="scale_factor_manager", tof_estimator=None, ext_tof_estimator=None):
        super().__init__(node_name)

        # Optional reference to a ToFScaleEstimator instance (same process).
        # None means ToF source is unavailable (e.g. simulation, or before
        # the node starts).  ScaleFactorManager never creates this itself -
        # main.py wires it in.
        self._tof_estimator = tof_estimator
        self._ext_tof_estimator = ext_tof_estimator

        # Reference to LiveScaler (same process), set post-construction via
        # set_live_scaler(). Needed by _publish_dense_depth_points to look
        # up the cumulative global alignment transform for dense points,
        # exactly mirroring what live_scaler already does for sparse points.
        # Cannot be set in __init__ because LiveScaler takes a reference to
        # THIS node - circular dependency; main.py wires it after both are
        # constructed.
        self._live_scaler = None

        # Runtime scale-source mode, initialised from constants but mutable
        # at runtime via cycle_scale_source_mode() (bound to 'm' key).
        self.scale_source_mode = constants.SCALE_SOURCE_MODE

        # Raw Depth-Anything-V2 results, stored separately from
        # current_scale_factors so the arbiter can always see both
        # sources independently regardless of the active mode.
        self._depth_scale_factors = {}

        self._topics = TopicManager(self)
        self._scale_factor_pub = self.create_publisher(
            Float64, "/tello_autonomy/scale_factor", 10
        )
        self._debug_pub = self.create_publisher(
            String, "/tello_autonomy/scale_source_debug", 10
        )
        self._dense_points_pub = self.create_publisher(
            PointCloud2, constants.TOPIC_DENSE_POINTS_METRIC, 10
        )
        self._dense_pose_pub = self.create_publisher(
            PoseStamped, "/tello_autonomy/dense_pose_metric", 10
        )

        self._frame_bridge = CvBridge()
        # timestamp -> frame_bgr, ordered by insertion (== arrival order, since
        # frames publish in increasing-timestamp order). Bounded on two axes:
        # age (evicted lazily on each insert) and count (hard cap, backstop).
        self._recent_frames = OrderedDict()

        self._topics.get_subscription(
            constants.TOPIC_RECENT_FRAME, Image, self._on_recent_frame
        )
        self._topics.get_subscription(
            constants.TOPIC_KEYFRAME_POINTS, PointCloud2, self._on_keyframe_points
        )
        self._topics.get_subscription(
            constants.TOPIC_MAP_TOPOLOGY_CHANGED, Int32, self._on_topology_changed
        )
        self._topics.get_subscription(
            constants.TOPIC_TRAJECTORY, Path, self._on_trajectory
        )

        # map_id -> list of PoseStamped from TOPIC_TRAJECTORY
        self._trajectory_cache = {}

        # map_id -> {"timer": rclpy Timer}
        self._map_state = {}

        # map_id -> list of row dicts (latest received snapshot for
        # that map_id) - same schema as before (keyframe_id, timestamp,
        # pixel_u, pixel_v, depth_camera_frame), map_id column added
        # back in only when building a request for the worker.
        self._map_points_cache = {}

        # map_id -> time.time() this node last received a
        # keyframe_points message for it. Used for staleness pruning.
        self._last_seen = {}

        # map_ids with a request currently queued or being processed by
        # the worker - prevents piling up redundant duplicate requests
        # for the same map_id while one is still in flight.
        self._in_flight_map_ids = set()

        # map_id -> most recently computed scale factor (float).
        # Instance attribute (not class-level) - each ScaleFactorManager
        # instance has its own independent dict.
        self.current_scale_factors = {}

        # map_id -> name of active scale source ("ext_tof", "internal_tof", or "depth")
        self._active_scale_sources = {}

        # pipeline_audit.md Finding #3: map_id -> {"candidate": float,
        # "confirmations": int} for a scale-factor jump that exceeded
        # constants.QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD relative to the
        # currently active value and has not yet been confirmed enough
        # times to be adopted. See _resolve_active_scale() /
        # _maybe_adopt_scale() for the actual quarantine logic. Absence
        # of a map_id here means "no quarantined candidate right now" -
        # either nothing large enough to quarantine has been seen, or a
        # previous quarantine was already resolved (adopted or
        # superseded).
        self._quarantine = {}
        
        # Callbacks
        self.on_scale_ready = None
        self.on_scale_computing = None
        self.on_scale_lost = None

        self._last_known_map_count = 0

        # ---- long-lived worker process: started once here, not once
        # per recompute. The model loads ONCE inside the worker process
        # and stays resident for this node's entire lifetime. ----
        self._request_queue = _MP_CONTEXT.Queue()
        self._result_queue = _MP_CONTEXT.Queue()
        self._worker_process = _MP_CONTEXT.Process(
            target=run_worker,
            args=(self._request_queue, self._result_queue),
            daemon=True,
        )
        self._worker_process.start()

        # Drains _result_queue on the ROS2 executor thread. Safe to run
        # frequently - get_nowait() never blocks, so this can never
        # cause the stall this whole redesign exists to eliminate.
        self._result_poll_timer = self.create_timer(
            constants.DEPTH_WORKER_RESULT_POLL_INTERVAL_SEC, self._poll_results
        )

        # Re-resolves the active scale factor for every tracked map_id at
        # the same cadence as the depth-result poll.  This is what lets
        # ToF's much faster updates actually reach current_scale_factors
        # in "auto"/"tof" mode without waiting for the next Depth-Anything
        # recompute cycle (which can take 30-60s).
        self._resolve_timer = self.create_timer(
            constants.DEPTH_WORKER_RESULT_POLL_INTERVAL_SEC, self._resolve_all_active_scales
        )

        self.get_logger().info(
            f"ScaleFactorManager: subscribed to {constants.TOPIC_KEYFRAME_POINTS} "
            f"and {constants.TOPIC_MAP_TOPOLOGY_CHANGED}. Depth inference worker "
            f"process started (pid={self._worker_process.pid})."
        )

    def set_live_scaler(self, live_scaler):
        """Wire in the LiveScaler reference post-construction (see __init__)."""
        self._live_scaler = live_scaler

    # ****************************************************************************************
    def _on_recent_frame(self, msg: Image):
        try:
            frame_bgr = self._frame_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert recent frame: {e}")
            return

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._recent_frames[timestamp] = frame_bgr
        self._evict_stale_frames()

    def _evict_stale_frames(self):
        now = time.time()
        # Age-based eviction: drop anything older than the max lookback
        # window - it can never be matched by a keyframe anymore anyway.
        while self._recent_frames:
            oldest_ts = next(iter(self._recent_frames))
            if (now - oldest_ts) <= constants.RECENT_FRAME_BUFFER_MAX_AGE_SEC:
                break
            self._recent_frames.popitem(last=False)

        # Hard count cap - backstop against age eviction falling behind a
        # publish-rate spike. OrderedDict + insertion-order publishing means
        # popping from the front always drops the oldest.
        while len(self._recent_frames) > constants.RECENT_FRAME_BUFFER_MAX_COUNT:
            self._recent_frames.popitem(last=False)

    def _find_recent_frame(self, target_timestamp, tolerance=None):
        """In-memory equivalent of perception.scale_factor.find_matching_frame()."""
        tolerance = tolerance if tolerance is not None else constants.KEYFRAME_MATCH_TOLERANCE_SEC
        best_ts, best_diff = None, None
        for ts in self._recent_frames:
            diff = abs(ts - target_timestamp)
            if best_diff is None or diff < best_diff:
                best_ts, best_diff = ts, diff
        if best_diff is not None and best_diff <= tolerance:
            return self._recent_frames[best_ts]
        return None

    # ****************************************************************************************
    def _on_keyframe_points(self, msg: PointCloud2):
        """
            One message = one full snapshot of a single map_id's
            current keyframe points, published periodically by the C++
            node's PublishLiveMapData(). Replaces this map_id's cached
            rows entirely (not appended).
        """
        map_id = _parse_map_id_from_frame_id(msg.header.frame_id)
        if map_id is None:
            self.get_logger().warn(
                f"keyframe_points message with unparseable frame_id '{msg.header.frame_id}' - ignoring."
            )
            return

        rows = []
        for kf_id, ts, u, v, depth in point_cloud2.read_points(
            msg, field_names=("keyframe_id", "keyframe_timestamp", "pixel_u", "pixel_v", "depth"),
            skip_nans=False,
        ):
            rows.append({
                "keyframe_id": int(kf_id),
                "timestamp": float(ts),
                "pixel_u": float(u),
                "pixel_v": float(v),
                "depth_camera_frame": float(depth),
            })

        is_new_map_id = map_id not in self._map_points_cache
        self._map_points_cache[map_id] = rows
        self._last_seen[map_id] = time.time()

        if is_new_map_id:
            self._start_tracking(map_id)

    # ****************************************************************************************
    def _on_topology_changed(self, msg):
        """
            Fired whenever the C++ node's set of active map_ids
            changes - exactly the right moment to check for map_ids
            whose keyframe_points messages have gone stale (merged
            away, C++ side stopped publishing for them).
        """
        old_count = self._last_known_map_count
        self._last_known_map_count = msg.data
        self._prune_stale_map_ids(old_count)

    # ****************************************************************************************
    def _on_trajectory(self, msg: Path):
        """
            Caches keyframe poses per map_id from TOPIC_TRAJECTORY.
            Used to position back-projected dense point clouds in the world frame.
        """
        map_id = _parse_map_id_from_frame_id(msg.header.frame_id)
        if map_id is not None:
            self._trajectory_cache[map_id] = msg.poses

    # ****************************************************************************************

    # ****************************************************************************************
    def _prune_stale_map_ids(self, prev_c_map_count=None):
        now = time.time()
        stale_map_ids = [
            map_id for map_id, last_seen in self._last_seen.items()
            if (now - last_seen) > constants.MAP_DATA_STALE_AFTER_SEC
        ]
        for map_id in stale_map_ids:
            self._stop_tracking(map_id)

        # Fire on_scale_lost ONCE after all pruning, using count direction
        # to distinguish loss from merge:
        #   count went DOWN  -> merge (maps consolidated) -> no alarm
        #   count same or UP -> loss (new map from restart) -> fire red
        if stale_map_ids and self.on_scale_lost is not None and prev_c_map_count is not None:
            if self._last_known_map_count >= prev_c_map_count:
                self.on_scale_lost()

    # ****************************************************************************************
    def _start_tracking(self, map_id):
        """
            Begins periodic recompute tracking for a newly-seen map_id:
            starts a repeating timer at constants.
            PERIODIC_RECALIBRATION_SECONDS, and attempts an immediate
            first dispatch rather than waiting a full interval for the
            first result.
        """
        self.get_logger().info(f"New map_id {map_id} detected - starting scale-factor tracking.")

        if map_id not in self.current_scale_factors and self.current_scale_factors:
            last_known_scale = list(self.current_scale_factors.values())[-1]
            if last_known_scale is not None and last_known_scale > 0.0:
                self.get_logger().info(
                    f"map_id {map_id}: seeding scale factor with previous map's scale ({last_known_scale:.2f})"
                )
                self.current_scale_factors[map_id] = last_known_scale
                self._active_scale_sources[map_id] = "seeded_prev_map"
                # FIX: the HUD only ever updates from on_scale_ready, which was
                # previously fired exclusively from _resolve_active_scale(). The
                # seed path bypassed that entirely, so manual_control's status
                # text stayed stuck on "Computing..." even though live_scaler
                # was already publishing metric poses/points for this map_id.
                if self.on_scale_ready is not None:
                    self.on_scale_ready(map_id, last_known_scale)

        timer = self.create_timer(
            constants.PERIODIC_RECALIBRATION_SECONDS,
            lambda: self._trigger_recompute(map_id),
        )
        self._map_state[map_id] = {"timer": timer}
        if self.on_scale_computing is not None:
            self.on_scale_computing(map_id)
        self._trigger_recompute(map_id)

    # ****************************************************************************************
    def _stop_tracking(self, map_id):
        """
            Drops all per-map_id state for a map_id that's gone stale
            (merged away) - cancels its timer and discards its cached
            scale factor, cached points, last-seen timestamp, and
            in-flight tracking.
        """
        state = self._map_state.pop(map_id, None)
        if state is not None and state["timer"] is not None:
            state["timer"].cancel()
        self.current_scale_factors.pop(map_id, None)
        self._active_scale_sources.pop(map_id, None)
        self._depth_scale_factors.pop(map_id, None)
        self._map_points_cache.pop(map_id, None)
        self._last_seen.pop(map_id, None)
        self._in_flight_map_ids.discard(map_id)
        self._quarantine.pop(map_id, None)
        if self._ext_tof_estimator is not None:
            self._ext_tof_estimator.reset_map(map_id)
        self.get_logger().info(f"map_id {map_id} no longer active - dropped its scale-factor state.")

    # ****************************************************************************************
    def _trigger_recompute(self, map_id):
        """
            Called from a ROS2 timer callback (the executor thread).
            Enqueues a request for the WORKER PROCESS and returns
            immediately - never runs inference itself, and never
            blocks. Skips enqueueing if a request for this map_id is
            already in flight (queued or currently being processed),
            rather than piling up redundant work the worker can't keep
            up with.
        """
        if map_id in self._in_flight_map_ids:
            self.get_logger().debug(
                f"map_id {map_id}: previous recompute still in flight - skipping this tick."
            )
            return

        rows = self._map_points_cache.get(map_id)
        if not rows:
            return

        num_keyframes = len({row["keyframe_id"] for row in rows})
        if num_keyframes < constants.MIN_KEYFRAMES_FOR_SCALE_FACTOR:
            self.get_logger().info(
                f"map_id {map_id}: only {num_keyframes} keyframes cached so far "
                f"(need {constants.MIN_KEYFRAMES_FOR_SCALE_FACTOR}) - skipping this tick."
            )
            return

        # NEW - attach matched frames for the keyframes the worker will use,
        # keyed by keyframe_id, instead of letting the worker cv2.imread()
        # them from disk (there is no disk copy anymore).
        keyframe_ids = sorted({row["keyframe_id"] for row in rows})
        recent_ids = keyframe_ids[-constants.SCALE_FACTOR_RECENT_KEYFRAME_COUNT:]
        matched_frames = {}
        for kf_id in recent_ids:
            kf_ts = next(r["timestamp"] for r in rows if r["keyframe_id"] == kf_id)
            frame = self._find_recent_frame(kf_ts)
            if frame is not None:
                matched_frames[kf_id] = frame

        self._in_flight_map_ids.add(map_id)
        self._request_queue.put({"map_id": map_id, "rows": rows, "matched_frames": matched_frames})

    # ****************************************************************************************
    def _poll_results(self):
        """
            Non-blocking drain of every result currently sitting in
            _result_queue. Runs on the ROS2 executor thread but never
            blocks - queue_module.Empty just means "nothing new yet,"
            not an error.
        """
        while True:
            try:
                result = self._result_queue.get_nowait()
            except queue_module.Empty:
                break

            map_id = result["map_id"]
            self._in_flight_map_ids.discard(map_id)

            if map_id not in self._map_state:
                # map_id went stale while this result was in flight -
                # discard rather than resurrect dropped state.
                continue

            if result.get("error"):
                self.get_logger().error(f"map_id {map_id}: worker raised: {result['error']}")
                continue

            if result["scale_factor"] is None:
                self.get_logger().warn(f"map_id {map_id}: no usable point ratios this attempt.")
                continue

            self._depth_scale_factors[map_id] = result["scale_factor"]
            self._resolve_active_scale(map_id)

            if result.get("dense_points_cam") is not None:
                self._publish_dense_depth_points(result)

            # on_scale_ready now fires from _resolve_active_scale, so it
            # always reflects whichever source actually won (ToF or Depth-AI).
            self.get_logger().info(
                f"map_id {map_id}: scale_factor={result['scale_factor']:.6f} "
                f"(from {result['num_points_used']} points across "
                f"{result['num_keyframes_used']} keyframes, "
                f"{result['num_keyframes_skipped']} skipped)"
            )

    # ****************************************************************************************
    def _publish_dense_depth_points(self, result: dict):
        """
            Transforms camera-frame dense points from Depth-Anything backprojection
            into the global world frame using the keyframe's pose, current metric
            scale factor, and the same cumulative map-alignment transform that
            live_scaler applies to the sparse stream. Previously this method only
            applied scale (local frame), so dense clouds for any map_id after the
            first re-init were silently mis-registered by up to several meters.
        """
        points_cam = result.get("dense_points_cam")
        map_id = result.get("map_id")
        kf_timestamp = result.get("dense_kf_timestamp")

        if points_cam is None or len(points_cam) == 0 or map_id is None:
            return

        active_scale = self.current_scale_factors.get(map_id)
        if not active_scale or active_scale <= 0:
            return

        # Dense points must go through the same global registration as
        # current_points_metric. Without this, every dense cloud published for
        # a map after the first re-init lands in that map's own local frame -
        # offset from the rest of the map by that submap's alignment vector
        # (up to several meters, per diagnostics). Refuse to publish rather
        # than publish something silently mis-registered.
        if self._live_scaler is None:
            return
        global_transform = self._live_scaler.get_global_transform(map_id)
        if global_transform is None:
            self.get_logger().debug(
                f"map_id {map_id}: global alignment not resolved yet - "
                f"holding dense-points publish until it is."
            )
            return
        R_glob, T_glob = global_transform

        poses = self._trajectory_cache.get(map_id, [])
        if not poses:
            return

        # Match by the actual keyframe's timestamp instead of blindly taking
        # the most recent trajectory pose. poses[-1] could be an entirely
        # different position than where this depth image was captured,
        # silently misplacing the dense cloud.
        matched_pose = self._find_pose_by_timestamp(poses, kf_timestamp)
        if matched_pose is None:
            self.get_logger().warn(
                f"map_id {map_id}: no trajectory pose found within tolerance "
                f"for dense depth keyframe (timestamp={kf_timestamp}) - "
                f"skipping this dense-points publish."
            )
            return

        raw_pos = matched_pose.pose.position
        orientation = matched_pose.pose.orientation

        # Local metric frame (same as current_pose_metric's un-aligned branch).
        translation_local_m = np.array(
            [raw_pos.x * active_scale, raw_pos.y * active_scale, raw_pos.z * active_scale],
            dtype=np.float64,
        )
        rotation_local = _quaternion_to_rotation_matrix(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # --- Local -> global, matching live_scaler exactly ---
        translation_global_m = R_glob @ translation_local_m + T_glob
        rotation_global = R_glob @ rotation_local
        qx, qy, qz, qw = _rotation_matrix_to_quat(rotation_global)

        # Points: camera-frame -> local metric world (rotation is scale-
        # invariant, so rotation_local via orientation is correct here) -> global.
        points_local_world = transform_camera_points_to_world(
            points_cam=points_cam,
            orientation=orientation,
            translation_m=tuple(translation_local_m),
        )
        points_world = (points_local_world @ R_glob.T) + T_glob

        frame_id_str = f"{constants.SLAM_MAP_FRAME_ID_PREFIX}{map_id}"
        now_stamp = self.get_clock().now().to_msg()

        dense_pose_msg = PoseStamped()
        dense_pose_msg.header.stamp = now_stamp
        dense_pose_msg.header.frame_id = frame_id_str
        dense_pose_msg.pose.position.x = float(translation_global_m[0])
        dense_pose_msg.pose.position.y = float(translation_global_m[1])
        dense_pose_msg.pose.position.z = float(translation_global_m[2])
        dense_pose_msg.pose.orientation.x = qx
        dense_pose_msg.pose.orientation.y = qy
        dense_pose_msg.pose.orientation.z = qz
        dense_pose_msg.pose.orientation.w = qw
        self._dense_pose_pub.publish(dense_pose_msg)

        header = dense_pose_msg.header
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points_world.tolist())
        self._dense_points_pub.publish(cloud_msg)
        self.get_logger().info(
            f"map_id {map_id}: Published {len(points_world)} dense metric points "
            f"to {constants.TOPIC_DENSE_POINTS_METRIC}."
        )

    # ****************************************************************************************
    @staticmethod
    def _find_pose_by_timestamp(poses, target_timestamp, tolerance_sec=0.05):
        """
        Finds the PoseStamped in `poses` whose header.stamp is closest to
        target_timestamp (a plain float, seconds - matches kf->mTimeStamp on
        the C++ side). Returns None if nothing is within tolerance, rather
        than falling back to the wrong pose.
        """
        if target_timestamp is None or not poses:
            return None
        best_pose, best_diff = None, None
        for pose_stamped in poses:
            stamp = pose_stamped.header.stamp
            pose_ts = stamp.sec + stamp.nanosec * 1e-9
            diff = abs(pose_ts - target_timestamp)
            if best_diff is None or diff < best_diff:
                best_pose, best_diff = pose_stamped, diff
        if best_diff is not None and best_diff <= tolerance_sec:
            return best_pose
        return None

    # ****************************************************************************************
    def _resolve_active_scale(self, map_id: int):
        """
            Picks the active scale factor for map_id from whichever
            source(s) are available, according to self.scale_source_mode,
            then routes it through _maybe_adopt_scale() (pipeline_audit.md
            Finding #3) before writing to self.current_scale_factors — the
            dict perception.live_scaler.LiveScaler reads. Nothing
            downstream needs to know which source won or whether a
            quarantine delay was involved.

            Also publishes to /tello_autonomy/scale_factor and fires
            on_scale_ready so the HUD overlay stays in sync regardless
            of whether a depth or ToF result triggered the call - but
            only once a value is actually adopted, not on every call
            that merely proposes one.
        """
        depth_scale = self._depth_scale_factors.get(map_id)
        tof_scale, tof_reliable = None, False
        if self._tof_estimator is not None:
            tof_scale, tof_reliable = self._tof_estimator.get_scale_estimate(map_id)

        ext_tof_scale_mm, ext_tof_reliable = None, False
        if self._ext_tof_estimator is not None:
            ext_tof_scale_mm, ext_tof_reliable = self._ext_tof_estimator.get_scale_estimate(map_id)
        ext_tof_scale = (ext_tof_scale_mm / 1000.0) if ext_tof_scale_mm is not None else None

        chosen_source = "none"
        mode = self.scale_source_mode
        if mode == "tof":
            # Prefer ToF; fall back to last depth result if ToF not ready yet.
            if tof_scale is not None:
                chosen = tof_scale
                chosen_source = "internal_tof"
            else:
                chosen = depth_scale
                chosen_source = "depth"
        elif mode == "depth":
            chosen = depth_scale
            chosen_source = "depth"
        else:  # "auto"
            # Prefer ToF when it's reliable (low std-dev, enough samples);
            # fall back to Depth-Anything otherwise.
            if tof_scale is not None and tof_reliable:
                chosen = tof_scale
                chosen_source = "internal_tof"
            else:
                chosen = depth_scale
                chosen_source = "depth"

        # ext_tof takes priority over WHATEVER the mode above selected,
        # whenever it's currently reliable - it's the most accurate available
        # source (+/-4mm) but only intermittently available (needs the floor
        # or an obstacle within ~1.2m below the drone).
        if ext_tof_scale is not None and ext_tof_reliable:
            chosen = ext_tof_scale
            chosen_source = "ext_tof"

        if chosen is None:
            return

        adopted_value = self._maybe_adopt_scale(map_id, chosen, chosen_source)
        if adopted_value is None:
            return  # candidate is quarantined - current_scale_factors untouched this call

        self.current_scale_factors[map_id] = adopted_value

        # Log scale governor switch or active governor
        previous_source = self._active_scale_sources.get(map_id)
        if previous_source != chosen_source:
            self.get_logger().info(
                f"ScaleFactorManager: [SCALE_SOURCE_SWITCH] map_id={map_id}: scale governor changed "
                f"'{previous_source}' -> '{chosen_source}' | active_scale={adopted_value:.6f}m "
                f"(ext_tof={ext_tof_scale}, int_tof={tof_scale}, depth={depth_scale})"
            )
            self._active_scale_sources[map_id] = chosen_source
        else:
            self.get_logger().info(
                f"ScaleFactorManager: [SCALE_SOURCE_ACTIVE] map_id={map_id}: active_scale={adopted_value:.6f}m "
                f"governed by '{chosen_source}' (ext_tof_reliable={ext_tof_reliable}, int_tof_reliable={tof_reliable}, mode={mode})"
            )

        _sf_msg = Float64()
        _sf_msg.data = adopted_value
        self._scale_factor_pub.publish(_sf_msg)

        dbg = String()
        dbg.data = (
            f"map_id={map_id} active_source={chosen_source} mode={mode} "
            f"tof_reliable={tof_reliable} ext_tof_reliable={ext_tof_reliable}"
        )
        self._debug_pub.publish(dbg)

        if self.on_scale_ready is not None:
            self.on_scale_ready(map_id, adopted_value)

    # ****************************************************************************************
    def _maybe_adopt_scale(self, map_id: int, candidate: float, source: str):
        """
            pipeline_audit.md Finding #3: gate between "a source proposed
            this scale factor" and "current_scale_factors[map_id] is
            actually updated to it". occupancy_map_cpp's OcTree already
            contains points inserted under whatever scale was active
            before this call - there's no mechanism reconciling old
            insertions with a corrected scale, so a large, sudden jump
            would silently smear/duplicate geometry in the live map.

            Returns the value that should be written to
            current_scale_factors[map_id] THIS call, or None if the
            candidate is being held in quarantine and nothing should be
            adopted yet.

            Sources are split into two confirmation styles:
              - "depth": requires QUARANTINE_CONFIRMATIONS_REQUIRED proposals
                that visibly differ from the previous one. Depth-Anything
                only produces a new number on a real recompute, so "did it
                change" is a meaningful signal of a new measurement.
              - "ext_tof" / "internal_tof": these are already smoothed /
                rate-limited at the source (see ExtTofScaleEstimator /
                ToFScaleEstimator). A genuinely-good reading returns the
                SAME float on every poll once locked on. Gating on "did it
                change" would starve these sources in quarantine forever.
                Instead, adopt once the candidate has persisted within
                threshold of itself for QUARANTINE_MIN_HOLD_SEC wall-clock
                seconds, regardless of whether the float literally changed
                between polls.

            Logic:
              - No active value yet: adopt immediately.
              - Within QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD of active: adopt
                immediately, clear any stale quarantine entry.
              - Sharp jump detected:
                  depth   -> change-based confirmation (unchanged logic).
                  tof     -> time-based confirmation (QUARANTINE_MIN_HOLD_SEC).
              - If the quarantined source changes between calls, reset the
                quarantine so the two styles' held state don't cross-contaminate.
        """
        active = self.current_scale_factors.get(map_id)

        if active is None or active == 0.0:
            return candidate  # nothing to compare against - adopt outright

        jump_ratio = abs(candidate - active) / abs(active)
        if jump_ratio <= constants.QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD:
            self._quarantine.pop(map_id, None)
            return candidate

        now = time.monotonic()
        is_time_based = source in ("ext_tof", "internal_tof")

        held = self._quarantine.get(map_id)

        # If the winning source changed (e.g. ext_tof took over from depth
        # mid-quarantine), the two styles' held state aren't comparable -
        # reset so we don't confirm a depth hold with a tof timer or vice versa.
        if held is not None and held.get("source") != source:
            held = None

        if held is None:
            entry = {"candidate": candidate, "source": source}
            if is_time_based:
                entry["first_seen"] = now
            else:
                entry["confirmations"] = 1
                entry["last_proposal"] = candidate
            self._quarantine[map_id] = entry
            self.get_logger().warn(
                f"map_id {map_id}: scale factor jump {active:.6f} -> {candidate:.6f} "
                f"({jump_ratio * 100:.1f}%) from '{source}' exceeds "
                f"{constants.QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD * 100:.0f}% threshold - quarantining."
            )
            return None

        held_candidate = held["candidate"]
        agreement_ratio = (
            abs(candidate - held_candidate) / abs(held_candidate) if held_candidate != 0.0 else 1.0
        )
        if agreement_ratio > constants.QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD:
            # The jump target itself moved - still noisy, restart quarantine.
            entry = {"candidate": candidate, "source": source}
            if is_time_based:
                entry["first_seen"] = now
            else:
                entry["confirmations"] = 1
                entry["last_proposal"] = candidate
            self._quarantine[map_id] = entry
            self.get_logger().warn(
                f"map_id {map_id}: quarantined candidate moved ({held_candidate:.6f} -> "
                f"{candidate:.6f}) - restarting quarantine for '{source}'."
            )
            return None

        # Candidate still agrees with the held one - check confirmation
        # using the style appropriate for this source.
        if is_time_based:
            held_duration = now - held.get("first_seen", now)
            if held_duration < constants.QUARANTINE_MIN_HOLD_SEC:
                return None
            self.get_logger().warn(
                f"map_id {map_id}: '{source}' jump to {candidate:.6f} held stable for "
                f"{held_duration:.1f}s (>= {constants.QUARANTINE_MIN_HOLD_SEC}s) - adopting."
            )
            self._quarantine.pop(map_id, None)
            return candidate

        # depth: original change-based confirmation - only count if the
        # float actually changed (i.e. a genuine new recompute arrived).
        last_proposal = held.get("last_proposal")
        if last_proposal is not None and candidate == last_proposal:
            return None  # fast timer re-asking about the same stale depth result

        held["last_proposal"] = candidate
        held["confirmations"] = held.get("confirmations", 1) + 1
        if held["confirmations"] < constants.QUARANTINE_CONFIRMATIONS_REQUIRED:
            self.get_logger().info(
                f"map_id {map_id}: quarantined jump to {candidate:.6f} confirmed "
                f"({held['confirmations']}/{constants.QUARANTINE_CONFIRMATIONS_REQUIRED})."
            )
            return None

        self.get_logger().warn(
            f"map_id {map_id}: scale factor jump to {candidate:.6f} confirmed "
            f"{held['confirmations']}x - adopting."
        )
        self._quarantine.pop(map_id, None)
        return candidate

    def _resolve_all_active_scales(self):
        """
            Fast-path re-resolve for every currently tracked map_id.
            Called by the resolve timer so that ToF's 10 Hz updates
            actually reach current_scale_factors in "auto"/"tof" mode
            rather than waiting until the next Depth-Anything result
            arrives (which can take 30-60s).
        """
        for map_id in list(self._map_state.keys()):
            self._resolve_active_scale(map_id)

    def cycle_scale_source_mode(self) -> str:
        """
            Cycles tof -> depth -> auto -> tof -> ...
            Called from main.py's 'm' key wiring via
            manual_control.on_scale_mode_cycle_requested.
            Returns the new mode string so the caller can update the HUD.
        """
        order = ["tof", "depth", "auto"]
        current = self.scale_source_mode
        idx = order.index(current) if current in order else 0
        self.scale_source_mode = order[(idx + 1) % len(order)]
        self.get_logger().info(
            f"ScaleFactorManager: scale source mode -> '{self.scale_source_mode}'"
        )
        # Immediately re-resolve all active map_ids so the switch takes
        # effect without waiting for the next sensor tick.
        self._resolve_all_active_scales()
        return self.scale_source_mode
    # ****************************************************************************************

    # ****************************************************************************************
    def destroy_node(self):
        """
            Cancels every per-map_id timer plus the result-poll timer,
            signals the worker process to exit via the sentinel value,
            joins it (falling back to terminate() if it doesn't exit
            cleanly), and cleans up the subscription before shutdown.
        """
        for map_id in list(self._map_state.keys()):
            self._stop_tracking(map_id)
        if self._result_poll_timer is not None:
            self._result_poll_timer.cancel()
        if self._resolve_timer is not None:
            self._resolve_timer.cancel()

        self._request_queue.put(None)  # sentinel - see depth_inference_worker.run_worker
        self._worker_process.join(timeout=5.0)
        if self._worker_process.is_alive():
            self.get_logger().warn("Depth inference worker didn't exit cleanly - terminating.")
            self._worker_process.terminate()

        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************
