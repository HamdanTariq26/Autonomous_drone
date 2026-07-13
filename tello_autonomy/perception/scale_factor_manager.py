"""
perception/scale_factor_manager.py

ROS2 node that decides WHEN to (re)compute a scale factor for each
active map_id, and calls perception.scale_factor +
perception.depth.depth_anything_v2 to actually do it.

CHANGED (bug fix): this used to read constants.LIVE_KEYFRAME_CSV, a
file the C++ node no longer writes - that file was replaced by two
topics, TOPIC_KEYFRAME_POINTS and TOPIC_TRAJECTORY, published
periodically (one message per active map_id, per tick) from
PublishLiveMapData() in mono_node_cpp.cpp. Nobody updated this node
when that change was made, so it was silently reading a stale leftover
file from a previous run - tracking a phantom map_id that was never
actually active this session, while never seeing the real active one.
This rewrite subscribes to TOPIC_KEYFRAME_POINTS directly instead,
caching the latest snapshot per map_id (each incoming message IS a
full snapshot for that map_id, same as the old CSV rewrite was for the
whole file each tick).

Map_id lifecycle is now tracked via message arrival, not a file read:
  - A map_id is "new" the first time a keyframe_points message for it
    arrives - starts periodic recompute tracking immediately.
  - A map_id is considered gone (merged away) if no keyframe_points
    message for it has arrived in longer than
    constants.MAP_DATA_STALE_AFTER_SEC - checked whenever
    TOPIC_MAP_TOPOLOGY_CHANGED fires, since that's exactly the signal
    that something about the active map set changed.

perception.scale_factor's public interface (compute_scale_factor_for_
recent_keyframes) is UNCHANGED - it still expects a DataFrame with a
map_id column and filters internally. This node builds that DataFrame
from its own per-map_id cache (adding the map_id column back in) rather
than reading a CSV, so scale_factor.py itself needed no changes.

Recomputation is dispatched onto its own worker thread (unchanged from
the previous fix) - never run inference on the ROS2 callback thread.
"""

import threading
import time

import pandas as pd
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from config import constants
from middleware.topic_manager import TopicManager
from perception import scale_factor
from perception.depth import depth_anything_v2


def _parse_map_id_from_frame_id(frame_id):
    """Same encoding used by perception.live_scaler - see that module for details."""
    if not frame_id.startswith(constants.SLAM_MAP_FRAME_ID_PREFIX):
        return None
    suffix = frame_id[len(constants.SLAM_MAP_FRAME_ID_PREFIX):]
    try:
        return int(suffix)
    except ValueError:
        return None


class ScaleFactorManager(Node):
    """
        Caches the latest keyframe_points snapshot per map_id (from
        TOPIC_KEYFRAME_POINTS) and runs a per-map_id periodic recompute
        timer. Current best scale factor per map_id is available via
        self.current_scale_factors (a plain dict, map_id -> float), for
        any other node/layer to read.
    """

    def __init__(self, node_name="scale_factor_manager"):
        super().__init__(node_name)

        self._topics = TopicManager(self)
        self._topics.get_subscription(
            constants.TOPIC_KEYFRAME_POINTS, PointCloud2, self._on_keyframe_points
        )
        self._topics.get_subscription(
            constants.TOPIC_MAP_TOPOLOGY_CHANGED, Bool, self._on_topology_changed
        )

        # map_id -> {"timer": rclpy Timer, "busy": threading.Event}
        self._map_state = {}

        # map_id -> list of row dicts (latest received snapshot for
        # that map_id). Each row: keyframe_id, timestamp, pixel_u,
        # pixel_v, depth_camera_frame - matches the old CSV's per-row
        # schema minus the map_id column (added back in when building
        # a DataFrame for scale_factor.py, since rows here are already
        # separated by map_id).
        self._map_points_cache = {}

        # map_id -> time.time() this node last received a
        # keyframe_points message for it. Used to detect staleness
        # (merged-away map_ids) - see _prune_stale_map_ids.
        self._last_seen = {}

        # map_id -> most recently computed scale factor (float).
        # Instance attribute (not class-level) - each ScaleFactorManager
        # instance must have its own independent dict.
        self.current_scale_factors = {}

        self.get_logger().info(
            f"ScaleFactorManager: subscribed to {constants.TOPIC_KEYFRAME_POINTS} "
            f"and {constants.TOPIC_MAP_TOPOLOGY_CHANGED}."
        )

    # ****************************************************************************************
    def _on_keyframe_points(self, msg: PointCloud2):
        """
            One message = one full snapshot of a single map_id's
            current keyframe points, published periodically by the C++
            node's PublishLiveMapData(). Replaces this map_id's cached
            rows entirely (not appended) - same semantics as the old
            CSV's full-rewrite-every-tick behavior.
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
            changes. There's no file to re-read anymore - instead,
            this is exactly the right moment to check for map_ids
            whose keyframe_points messages have gone stale (i.e. they
            were merged away and the C++ side stopped publishing for
            them).
        """
        self._prune_stale_map_ids()

    # ****************************************************************************************
    def _prune_stale_map_ids(self):
        now = time.time()
        stale_map_ids = [
            map_id for map_id, last_seen in self._last_seen.items()
            if (now - last_seen) > constants.MAP_DATA_STALE_AFTER_SEC
        ]
        for map_id in stale_map_ids:
            self._stop_tracking(map_id)
    # ****************************************************************************************

    # ****************************************************************************************
    def _start_tracking(self, map_id):
        """
            Begins periodic recompute tracking for a newly-seen map_id:
            starts a repeating timer at constants.
            PERIODIC_RECALIBRATION_SECONDS, and dispatches an immediate
            first computation attempt rather than waiting a full
            interval for the first result.
        """
        self.get_logger().info(f"New map_id {map_id} detected - starting scale-factor tracking.")
        timer = self.create_timer(
            constants.PERIODIC_RECALIBRATION_SECONDS,
            lambda: self._trigger_recompute(map_id),
        )
        self._map_state[map_id] = {"timer": timer, "busy": threading.Event()}
        self._trigger_recompute(map_id)

    # ****************************************************************************************
    def _stop_tracking(self, map_id):
        """
            Drops all per-map_id state for a map_id that's gone stale
            (merged away) - cancels its timer and discards its cached
            scale factor, cached points, and last-seen timestamp.
        """
        state = self._map_state.pop(map_id, None)
        if state is not None and state["timer"] is not None:
            state["timer"].cancel()
        self.current_scale_factors.pop(map_id, None)
        self._map_points_cache.pop(map_id, None)
        self._last_seen.pop(map_id, None)
        self.get_logger().info(f"map_id {map_id} no longer active - dropped its scale-factor state.")

    # ****************************************************************************************
    def _trigger_recompute(self, map_id):
        """
            Called from a ROS2 timer callback (this node's single
            executor thread). Dispatches the actual (slow, CPU-bound)
            recomputation onto its own worker thread and returns
            immediately - see _recompute_worker.
        """
        state = self._map_state.get(map_id)
        if state is None:
            return  # map_id went stale between scheduling and firing

        if state["busy"].is_set():
            self.get_logger().debug(
                f"map_id {map_id}: previous recompute still running - skipping this tick."
            )
            return

        state["busy"].set()
        threading.Thread(
            target=self._recompute_worker, args=(map_id, state["busy"]), daemon=True
        ).start()

    # ****************************************************************************************
    def _recompute_worker(self, map_id, busy_flag):
        try:
            self._recompute_for_map_id(map_id)
        except Exception as e:
            self.get_logger().error(f"map_id {map_id}: recompute worker raised: {e}")
        finally:
            busy_flag.clear()

    # ****************************************************************************************
    def _recompute_for_map_id(self, map_id):
        """
            Attempts a (re)computation for map_id using its cached
            keyframe_points rows. No-ops (not an error) if map_id has
            fewer than constants.MIN_KEYFRAMES_FOR_SCALE_FACTOR
            keyframes cached so far, or if no usable point ratios were
            collected this attempt.

            Runs on the worker thread dispatched by _trigger_recompute -
            never call this directly from a ROS2 callback.
        """
        rows = self._map_points_cache.get(map_id)
        if not rows:
            return

        # Rebuild the DataFrame shape scale_factor.py expects (same
        # columns as the old CSV, map_id column added back in since
        # scale_factor.py's public interface still filters by it).
        df = pd.DataFrame(rows)
        df["map_id"] = map_id

        num_keyframes = df["keyframe_id"].nunique()
        if num_keyframes < constants.MIN_KEYFRAMES_FOR_SCALE_FACTOR:
            self.get_logger().info(
                f"map_id {map_id}: only {num_keyframes} keyframes cached so far "
                f"(need {constants.MIN_KEYFRAMES_FOR_SCALE_FACTOR}) - skipping this tick."
            )
            return

        frame_index = scale_factor.build_frame_index(constants.SAVE_FRAMES_DIR)

        result = scale_factor.compute_scale_factor_for_recent_keyframes(
            keyframe_df=df,
            map_id=map_id,
            frame_index=frame_index,
            infer_metric_depth_fn=depth_anything_v2.infer_metric_depth,
        )

        if result["scale_factor"] is None:
            self.get_logger().warn(f"map_id {map_id}: no usable point ratios this attempt.")
            return

        if map_id not in self._map_state:
            self.get_logger().info(
                f"map_id {map_id}: recompute finished but map_id is no longer tracked - discarding result."
            )
            return

        self.current_scale_factors[map_id] = result["scale_factor"]
        self.get_logger().info(
            f"map_id {map_id}: scale_factor={result['scale_factor']:.6f} "
            f"(from {result['num_points_used']} points across "
            f"{result['num_keyframes_used']} keyframes, "
            f"{result['num_keyframes_skipped']} skipped)"
        )

    # ****************************************************************************************
    def destroy_node(self):
        """Cancels every per-map_id timer and cleans up the subscription before shutdown."""
        for map_id in list(self._map_state.keys()):
            self._stop_tracking(map_id)
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************