"""
perception/live_scaler.py

Subscribes to the C++ SLAM node's raw per-frame pose (TOPIC_CURRENT_POSE_RAW)
and tracked-points cloud (TOPIC_CURRENT_POINTS_RAW) - both in ORB-SLAM3's
arbitrary SLAM units - looks up the current scale factor for whichever
map_id each message belongs to (encoded in header.frame_id, see
config.constants.SLAM_MAP_FRAME_ID_PREFIX), multiplies every position by
it, and republishes metric-scale versions on TOPIC_CURRENT_POSE_METRIC /
TOPIC_CURRENT_POINTS_METRIC.

Design choice: takes a direct Python reference to a ScaleFactorManager
instance (both nodes run in the same process - see scripts/main.py)
rather than a ROS2 service/topic round-trip to ask "what's the scale
factor for this map_id". Simpler and lower-latency for a same-process
pair; the tradeoff is these two nodes can't be split across separate
processes without revisiting this (a service call would be needed
instead - see middleware/service_manager.py's stub for where that
would go if this ever becomes necessary).

If no scale factor exists yet for a message's map_id, that message is
DROPPED, not published unscaled - a consumer of the _METRIC topics
must be able to trust that anything they receive there really is in
meters. Orientation (rotation) is never touched - scale only affects
position, not orientation.

Map alignment (TOPIC_MAP_ALIGNMENT):
  LiveScaler is also responsible for computing and publishing the
  alignment transform whenever SLAM re-initializes with a new map_id.
  It does this from RAW pose data, which is the only correct place to
  do it: both anchor poses (last old-map and first new-map) share the
  same arbitrary SLAM scale, so the computed offset is a real geometric
  quantity in a consistent unit system. The offset is then scaled by
  the NEW map's scale factor to produce a metric-space transform that
  occupancy_map_cpp can apply directly to new-map metric points.

  Deferred computation:
    Anchor poses are captured immediately at transition time (the exact
    moment the new map_id first appears). However, alignment calculation
    is deferred until the new map's scale factor actually lands. Once available,
    the alignment transform is computed and published on TOPIC_MAP_ALIGNMENT.
    If the scale factor does not land within MAX_ALIGNMENT_GAP_SEC, a rejection
    is published to trigger a clean hard-reset in occupancy_map_cpp.
"""

import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.node import Node

from config import constants
from middleware.topic_manager import TopicManager
from perception.pose_transform import (
    _quaternion_to_rotation_matrix,
    _rotation_matrix_to_quat,
)
from tello_autonomy_msgs.msg import MapAlignment


def parse_map_id_from_frame_id(frame_id):
    """
        Reverses the encoding used on the C++ side: header.frame_id is
        set to SLAM_MAP_FRAME_ID_PREFIX + str(map_id) (e.g. "slam_map_3").
        Returns the map_id as an int, or None if frame_id doesn't match
        the expected prefix/format (should never happen if the C++ side
        and this stay in sync - treated as "drop this message", not an
        error, since a single malformed header shouldn't crash the node).
    """
    if not frame_id.startswith(constants.SLAM_MAP_FRAME_ID_PREFIX):
        return None
    suffix = frame_id[len(constants.SLAM_MAP_FRAME_ID_PREFIX):]
    try:
        return int(suffix)
    except ValueError:
        return None


class LiveScaler(Node):
    def __init__(self, scale_factor_manager, node_name="live_scaler"):
        """
            scale_factor_manager: a perception.scale_factor_manager.ScaleFactorManager
                instance (already constructed, same process) - read via
                its .current_scale_factors dict, never mutated here.
        """
        super().__init__(node_name)
        self._scale_factor_manager = scale_factor_manager
        self._topics = TopicManager(self)

        self._pose_pub = self._topics.get_publisher(constants.TOPIC_CURRENT_POSE_METRIC, PoseStamped)
        self._points_pub = self._topics.get_publisher(constants.TOPIC_CURRENT_POINTS_METRIC, PointCloud2)
        self._alignment_pub = self._topics.get_publisher(constants.TOPIC_MAP_ALIGNMENT, MapAlignment)

        self._topics.get_subscription(constants.TOPIC_CURRENT_POSE_RAW, PoseStamped, self._on_pose_raw)
        self._topics.get_subscription(constants.TOPIC_CURRENT_POINTS_RAW, PointCloud2, self._on_points_raw)

        self._dropped_no_scale_factor = 0  # simple counter, logged periodically rather than every drop

        # --- Per-map_id raw-pose anchor for alignment computation ---
        # Track the most recent RAW pose seen while each map_id was current,
        # updated on every raw pose callback (same "whichever turns out to be
        # last" pattern occupancy_map_cpp used for its own anchors, but here
        # operating on RAW poses so the alignment math stays in one consistent
        # unit system).
        self._current_raw_map_id = None
        self._last_raw_pose = None          # (x, y, z) in raw SLAM units
        self._last_raw_orientation = None   # geometry_msgs Quaternion
        self._last_raw_pose_time = None     # time.monotonic() timestamp

        # Monotonically increasing guard: ORB-SLAM3 map_ids never legitimately
        # go backward, so a transition to an already-seen id is cross-topic
        # noise (dense vs regular stream racing), not a real re-init event.
        self._map_ids_already_aligned = set()

        # Pending transitions waiting on a scale factor before alignment
        # can be computed. Keyed by new_map_id.
        self._pending_transitions = {}

        # map_id -> (rot_matrix_3x3, trans_vec_3) cumulative transform relative
        # to global Map 0. Keeps all subsequent submaps (0 -> 1 -> 2 -> 3)
        # registered into a single global metric coordinate system.
        self._active_map_transforms = {}

        self.get_logger().info("LiveScaler: subscribed to raw pose/points, ready to scale.")

    # ****************************************************************************************
    def get_global_transform(self, map_id):
        """
        Returns (R_glob, T_glob) - the SAME cumulative rotation/translation
        this node applies to register map_id's points into the global
        Map 0 frame - or None if map_id is a submap (>0) whose alignment
        hasn't resolved yet. Read-only accessor for
        perception/scale_factor_manager.py's dense depth backprojection,
        which needs to apply this exact transform too - previously it
        didn't, so dense points for any map after the first re-init landed
        in that map's own LOCAL frame instead of the global one.
        """
        if map_id in self._active_map_transforms:
            return self._active_map_transforms[map_id]
        if map_id == 0:
            return (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
        return None
    # ****************************************************************************************

    # ****************************************************************************************
    def _get_scale_factor(self, frame_id):
        map_id = parse_map_id_from_frame_id(frame_id)
        if map_id is None:
            return None
        return self._scale_factor_manager.current_scale_factors.get(map_id)

    def _note_dropped(self, reason):
        self._dropped_no_scale_factor += 1
        if self._dropped_no_scale_factor % 100 == 1:  # log occasionally, not every single drop
            self.get_logger().info(
                f"LiveScaler: dropped {self._dropped_no_scale_factor} message(s) so far ({reason})"
            )

    def _note_depth_filtered(self, count):
        self._depth_filtered_total = getattr(self, "_depth_filtered_total", 0) + count
        if self._depth_filtered_total % 500 < count:  # log occasionally, not every message
            self.get_logger().info(
                f"LiveScaler: filtered {self._depth_filtered_total} near-degenerate-depth "
                f"point(s) so far (depth <= {constants.MIN_LIVE_POINT_DEPTH_M}m)"
            )
    # ****************************************************************************************

    # ****************************************************************************************
    def _register_transition(self, new_map_id, msg):
        """
        Called at the FIRST raw pose seen for a new map_id. Captures both
        anchors immediately (old map's last known pose, new map's first pose)
        so they remain a snapshot of the exact re-initialization moment.
        Calculation is deferred until a scale factor is ready.
        """
        if new_map_id in self._map_ids_already_aligned or new_map_id in self._pending_transitions:
            return

        if self._last_raw_pose is None:
            self._map_ids_already_aligned.add(new_map_id)
            self.get_logger().warn(
                f"Map transition -> {new_map_id}: no prior raw anchor - cannot align."
            )
            self._publish_rejection(self._current_raw_map_id, new_map_id, 0.0, 0.0)
            return

        self._pending_transitions[new_map_id] = {
            "old_map_id": self._current_raw_map_id,
            "old_pos": np.array(self._last_raw_pose, dtype=np.float64),
            "old_quat": self._last_raw_orientation,
            # old_scale must be captured NOW, not at resolution time: _stop_tracking
            # can prune current_scale_factors[old_map_id] after MAP_DATA_STALE_AFTER_SEC
            # (3 s), but alignment resolution is deferred up to MAX_ALIGNMENT_GAP_SEC
            # (25 s). Looking up old_scale at resolution time would therefore often
            # return None for any non-trivial gap.
            "old_scale": self._scale_factor_manager.current_scale_factors.get(
                self._current_raw_map_id
            ),
            "new_pos": np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                dtype=np.float64,
            ),
            "new_quat": msg.pose.orientation,
            "registered_at": time.monotonic(),
        }
        self.get_logger().info(
            f"Map transition {self._current_raw_map_id} -> {new_map_id}: "
            f"anchors captured, waiting for scale factor to compute alignment."
        )

    def _try_resolve_pending_transitions(self):
        """
        Checks all pending transitions to see if their scale factors have arrived.
        If ready, computes and publishes the alignment transform.
        If waiting exceeds MAX_ALIGNMENT_GAP_SEC, times out and publishes a rejection.
        """
        if not self._pending_transitions:
            return

        resolved = []
        now = time.monotonic()

        for new_map_id, anchor in self._pending_transitions.items():
            scale = self._scale_factor_manager.current_scale_factors.get(new_map_id)
            if scale is None or scale <= 0.0:
                waited = now - anchor["registered_at"]
                if waited > constants.MAX_ALIGNMENT_GAP_SEC:
                    self._publish_rejection(
                        anchor["old_map_id"], new_map_id, 0.0, waited
                    )
                    self._map_ids_already_aligned.add(new_map_id)
                    resolved.append(new_map_id)
                    self.get_logger().warn(
                        f"Map transition -> {new_map_id}: scale factor didn't "
                        f"arrive within {constants.MAX_ALIGNMENT_GAP_SEC}s - giving up, hard-reset."
                    )
                continue

            self._compute_and_publish_alignment(new_map_id, anchor, scale)
            self._map_ids_already_aligned.add(new_map_id)
            resolved.append(new_map_id)

        for map_id in resolved:
            del self._pending_transitions[map_id]

    def _compute_and_publish_alignment(self, new_map_id, anchor, new_scale):
        """
        Computes the metric alignment transform from the new map's raw frame
        to the old map's metric frame.

        Correct formula (see module docstring and the bug analysis in the
        commit message):

          T = (old_pos_raw * old_scale) - R_new_to_old @ (new_pos_raw * new_scale)

        Both anchors must use THEIR OWN map's scale factor. Applying new_scale
        to both (the previous bug) introduces an error of:

          old_pos_raw * (new_scale - old_scale)

        which grows with flight distance and scale-factor disagreement - easily
        >1 m, comparable to the MAX_ALIGNMENT_TRANSLATION_M = 2.0 m rejection
        threshold, causing legitimate alignments to flip to hard-resets.

        old_scale is captured at transition time (_register_transition) rather
        than looked up here, because the old map may already have been pruned
        from current_scale_factors by the time this resolves.
        """
        old_scale = anchor.get("old_scale")
        if old_scale is None or old_scale <= 0.0:
            # Old map's scale was never known (first map ever, or already pruned
            # before we had a chance to snapshot it). We cannot correctly mix the
            # two raw unit systems - reject rather than silently produce a wrong
            # translation.
            self.get_logger().warn(
                f"Map alignment {anchor['old_map_id']} -> {new_map_id}: "
                f"old map's scale factor unavailable - cannot compute correct "
                f"metric translation. Publishing rejection (hard-reset)."
            )
            self._publish_rejection(
                anchor["old_map_id"], new_map_id, 0.0,
                time.monotonic() - anchor["registered_at"]
            )
            return

        old_q = anchor["old_quat"]
        new_q = anchor["new_quat"]

        rot_old = _quaternion_to_rotation_matrix(old_q.x, old_q.y, old_q.z, old_q.w)
        rot_new = _quaternion_to_rotation_matrix(new_q.x, new_q.y, new_q.z, new_q.w)
        rot_new_to_old = rot_old @ rot_new.T

        # Each anchor term is scaled by ITS OWN map's scale factor before
        # combining. Both results are already in metric units.
        trans_metric = (
            anchor["old_pos"] * old_scale
            - rot_new_to_old @ (anchor["new_pos"] * new_scale)
        )

        # Compound with old map's cumulative global transform so that map transitions
        # (0 -> 1 -> 2 -> 3) remain registered to global Map 0 space rather than
        # drifting or resetting relative to the immediately preceding submap.
        old_map_id = anchor["old_map_id"]
        if old_map_id in self._active_map_transforms:
            rot_old_glob, trans_old_glob = self._active_map_transforms[old_map_id]
        else:
            rot_old_glob = np.eye(3, dtype=np.float64)
            trans_old_glob = np.zeros(3, dtype=np.float64)

        rot_new_glob = rot_old_glob @ rot_new_to_old
        trans_new_glob = rot_old_glob @ trans_metric + trans_old_glob

        offset_m = float(np.linalg.norm(trans_metric))
        gap_sec = time.monotonic() - anchor["registered_at"]

        msg_out = MapAlignment()
        msg_out.old_map_id = anchor["old_map_id"] if anchor["old_map_id"] is not None else -1
        msg_out.new_map_id = new_map_id
        msg_out.offset_m = offset_m
        msg_out.gap_sec = gap_sec

        offset_ok = offset_m <= constants.MAX_ALIGNMENT_TRANSLATION_M
        gap_ok = gap_sec <= constants.MAX_ALIGNMENT_GAP_SEC
        msg_out.accepted = offset_ok and gap_ok

        if msg_out.accepted:
            self._active_map_transforms[new_map_id] = (rot_new_glob, trans_new_glob)
            msg_out.translation.x = float(trans_new_glob[0])
            msg_out.translation.y = float(trans_new_glob[1])
            msg_out.translation.z = float(trans_new_glob[2])
            qx, qy, qz, qw = _rotation_matrix_to_quat(rot_new_glob)
            msg_out.rotation.x = qx
            msg_out.rotation.y = qy
            msg_out.rotation.z = qz
            msg_out.rotation.w = qw
            self.get_logger().warn(
                f"Map alignment {anchor['old_map_id']} -> {new_map_id}: "
                f"offset {offset_m:.3f}m over {gap_sec:.2f}s (scale={new_scale:.3f}) - accepted."
            )
        else:
            # Rejection means the OctoMap will hard-reset to a fresh origin for new_map_id.
            # The new map's physical "global origin" is therefore the drone's current metric
            # position in the old map's coordinate frame. Store the old map's cumulative
            # global transform as the seed for new_map_id so that any subsequent accepted
            # transitions (new_map_id -> N+1) still compound correctly against global Map 0
            # rather than silently falling back to identity and breaking the coordinate chain.
            self._active_map_transforms[new_map_id] = (rot_old_glob, trans_old_glob)
            self.get_logger().warn(
                f"Map alignment {anchor['old_map_id']} -> {new_map_id}: "
                f"offset {offset_m:.3f}m / gap {gap_sec:.2f}s exceeds bounds - rejected. "
                f"Seeding new_map_id={new_map_id} with parent global transform to preserve chain."
            )

        self._alignment_pub.publish(msg_out)

    def _publish_rejection(self, old_map_id, new_map_id, offset_m, gap_sec):
        msg_out = MapAlignment()
        msg_out.old_map_id = old_map_id if old_map_id is not None else -1
        msg_out.new_map_id = new_map_id
        msg_out.accepted = False
        msg_out.offset_m = offset_m
        msg_out.gap_sec = gap_sec
        self._alignment_pub.publish(msg_out)
    # ****************************************************************************************

    # ****************************************************************************************
    def _on_pose_raw(self, msg: PoseStamped):
        map_id = parse_map_id_from_frame_id(msg.header.frame_id)

        if map_id is not None:
            if self._current_raw_map_id is not None and map_id != self._current_raw_map_id:
                self._register_transition(map_id, msg)

            self._current_raw_map_id = map_id
            self._last_raw_pose = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )
            self._last_raw_orientation = msg.pose.orientation
            self._last_raw_pose_time = time.monotonic()

            # Every raw pose attempt to resolve pending transitions once scale factor arrives
            self._try_resolve_pending_transitions()

        # --- Scaling and transform for metric republish ---
        scale = self._get_scale_factor(msg.header.frame_id)
        if scale is None:
            self._note_dropped("no scale factor yet for this map_id")
            return

        scaled = PoseStamped()
        scaled.header = msg.header

        if map_id is not None and map_id in self._active_map_transforms:
            R_glob, T_glob = self._active_map_transforms[map_id]
            pos_metric = np.array([msg.pose.position.x * scale, msg.pose.position.y * scale, msg.pose.position.z * scale], dtype=np.float64)
            pos_glob = R_glob @ pos_metric + T_glob

            q_raw = msg.pose.orientation
            rot_raw = _quaternion_to_rotation_matrix(q_raw.x, q_raw.y, q_raw.z, q_raw.w)
            rot_glob = R_glob @ rot_raw
            qx, qy, qz, qw = _rotation_matrix_to_quat(rot_glob)

            scaled.pose.position.x = float(pos_glob[0])
            scaled.pose.position.y = float(pos_glob[1])
            scaled.pose.position.z = float(pos_glob[2])
            scaled.pose.orientation.x = qx
            scaled.pose.orientation.y = qy
            scaled.pose.orientation.z = qz
            scaled.pose.orientation.w = qw
            self._pose_pub.publish(scaled)
        elif map_id == 0:
            # Map 0 IS the global frame (identity transform) - never gets
            # an entry in _active_map_transforms because no transition
            # ever produced it, not because it's unregistered.
            scaled.pose.position.x = msg.pose.position.x * scale
            scaled.pose.position.y = msg.pose.position.y * scale
            scaled.pose.position.z = msg.pose.position.z * scale
            scaled.pose.orientation = msg.pose.orientation
            self._pose_pub.publish(scaled)
        else:
            # map_id > 0 but not yet in _active_map_transforms: this
            # submap's alignment hasn't resolved yet. Publishing a
            # scale-only (local-frame) pose here used to go out on
            # current_pose_metric looking exactly like a globally
            # registered one - occupancy_map_cpp has its own gate against
            # this, but mission_controller's relay to exploration_cpp's
            # RRT root does not, and that's what let the RRT tree plant
            # its root position meters away from where the (correctly
            # withheld) OcTree geometry actually was, right after a
            # re-init. Drop instead - a brief gap in current_pose_metric
            # during the ~1s alignment window is far safer than a
            # silently mis-registered pose.
            self._note_dropped(f"map_id {map_id} not yet globally aligned")
    # ****************************************************************************************

    # ****************************************************************************************
    def _on_points_raw(self, msg: PointCloud2):
        map_id = parse_map_id_from_frame_id(msg.header.frame_id)
        scale = self._get_scale_factor(msg.header.frame_id)
        if scale is None:
            self._note_dropped("no scale factor yet for this map_id")
            return

        structured = np.array(
            list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        )
        if structured.size == 0:
            return

        points = np.stack(
            [structured["x"], structured["y"], structured["z"]], axis=-1
        ).astype(np.float64)

        scaled_points = points * scale

        depth_mask = scaled_points[:, 2] > constants.MIN_LIVE_POINT_DEPTH_M
        num_rejected = scaled_points.shape[0] - int(depth_mask.sum())
        if num_rejected > 0:
            self._note_depth_filtered(num_rejected)
        scaled_points = scaled_points[depth_mask]

        if scaled_points.shape[0] == 0:
            return

        # Transform points to global Map 0 coordinate frame if alignment
        # exists; otherwise (map_id > 0, not yet aligned) drop rather than
        # publish local-frame points under the global topic name - same
        # rationale as _on_pose_raw above.
        if map_id is not None and map_id in self._active_map_transforms:
            R_glob, T_glob = self._active_map_transforms[map_id]
            scaled_points = (scaled_points @ R_glob.T) + T_glob
        elif map_id != 0:
            self._note_dropped(f"map_id {map_id} not yet globally aligned")
            return

        scaled_msg = point_cloud2.create_cloud_xyz32(msg.header, scaled_points.tolist())
        self._points_pub.publish(scaled_msg)
    # ****************************************************************************************

    # ****************************************************************************************
    def destroy_node(self):
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************
