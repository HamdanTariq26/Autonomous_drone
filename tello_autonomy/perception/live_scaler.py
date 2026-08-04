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
"""

import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.node import Node

from config import constants
from middleware.topic_manager import TopicManager


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

        self._topics.get_subscription(constants.TOPIC_CURRENT_POSE_RAW, PoseStamped, self._on_pose_raw)
        self._topics.get_subscription(constants.TOPIC_CURRENT_POINTS_RAW, PointCloud2, self._on_points_raw)

        self._dropped_no_scale_factor = 0  # simple counter, logged periodically rather than every drop

        self.get_logger().info("LiveScaler: subscribed to raw pose/points, ready to scale.")

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
    def _on_pose_raw(self, msg: PoseStamped):
        scale = self._get_scale_factor(msg.header.frame_id)
        if scale is None:
            self._note_dropped("no scale factor yet for this map_id")
            return

        scaled = PoseStamped()
        scaled.header = msg.header
        scaled.pose.position.x = msg.pose.position.x * scale
        scaled.pose.position.y = msg.pose.position.y * scale
        scaled.pose.position.z = msg.pose.position.z * scale
        scaled.pose.orientation = msg.pose.orientation  # rotation unaffected by scale

        self._pose_pub.publish(scaled)
    # ****************************************************************************************

    # ****************************************************************************************
    def _on_points_raw(self, msg: PointCloud2):
        scale = self._get_scale_factor(msg.header.frame_id)
        if scale is None:
            self._note_dropped("no scale factor yet for this map_id")
            return

        structured = np.array(
            list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        )
        if structured.size == 0:
            return

        # CHANGED (fix): read_points returns a structured array with named
        # fields (x, y, z as float32) - it can't be multiplied by a scalar
        # directly (numpy raises _UFuncNoLoopError). Stack the named fields
        # into a plain (N, 3) float64 array first.
        points = np.stack(
            [structured["x"], structured["y"], structured["z"]], axis=-1
        ).astype(np.float64)

        scaled_points = points * scale

        # Reject points with near-degenerate triangulation depth. Z = forward
        # in this project's SLAM convention (see trajectory_tracker.py's
        # coordinate-frame docstring). Points at or below this depth are
        # almost always triangulation noise near the epipole, not real
        # nearby geometry - see MIN_LIVE_POINT_DEPTH_M's comment in
        # config/constants.py.
        depth_mask = scaled_points[:, 2] > constants.MIN_LIVE_POINT_DEPTH_M
        num_rejected = scaled_points.shape[0] - int(depth_mask.sum())
        if num_rejected > 0:
            self._note_depth_filtered(num_rejected)
        scaled_points = scaled_points[depth_mask]

        if scaled_points.shape[0] == 0:
            return

        scaled_msg = point_cloud2.create_cloud_xyz32(msg.header, scaled_points.tolist())
        self._points_pub.publish(scaled_msg)
    # ****************************************************************************************

    # ****************************************************************************************
    def destroy_node(self):
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************
