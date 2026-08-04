"""
middleware/topic_manager.py

Centralizes creation of every ROS2 publisher and subscriber used by this
package, along with the QoS profile each topic must use. No other file
should call node.create_publisher()/create_subscription() directly -
route it through this module instead, so QoS decisions live in exactly
one place (architectural principle #2/#3 in the handoff doc).

QoS profiles here are matched to what the C++ mono_node_cpp.cpp node
expects. Do not "simplify" these away - see handoff doc Section 7.
"""

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from config import constants


# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------

# Image / timestep topics exchanged at the SLAM node's publish rate.
# Must match mono_node_cpp.cpp's subscriber QoS exactly - RELIABLE +
# KEEP_LAST + depth=1. Do not change without also changing the C++ side.
SLAM_STREAM_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Handshake topics (experiment settings + ack). These fire once at
# startup, before the other side's subscriber/publisher is guaranteed to
# have been discovered by DDS. TRANSIENT_LOCAL durability means a
# late-discovered subscriber still receives the last published message,
# fixing the "works on 2nd ros2 run, not the 1st" issue documented in
# the handoff doc (Section 7, ros_bridge.py design intent).
HANDSHAKE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# General-purpose topics with no special timing/discovery requirements
# (keyframe timestamps, map topology change notifications, and future
# reserved topics like occupancy grid / frontier goals / planned path /
# mission status). Reliable delivery, small buffer.
DEFAULT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Maps each known topic constant to the QoS profile it must use.
# Add new topics here as they're introduced - never inline a QoS
# profile in ros_bridge.py or anywhere else.
_TOPIC_QOS = {
    constants.TOPIC_EXPERIMENT_SETTINGS: HANDSHAKE_QOS,
    constants.TOPIC_EXP_SETTINGS_ACK: HANDSHAKE_QOS,
    constants.TOPIC_IMG_MSG: SLAM_STREAM_QOS,
    constants.TOPIC_TIMESTEP_MSG: SLAM_STREAM_QOS,
    constants.TOPIC_RECENT_FRAME: SLAM_STREAM_QOS,  # same cadence/reliability as IMG_MSG
    constants.TOPIC_KEYFRAME_TIMESTAMPS: DEFAULT_QOS,
    constants.TOPIC_MAP_TOPOLOGY_CHANGED: DEFAULT_QOS,
    # NEW - per-frame pose/points stream, same cadence/reliability
    # needs as the image/timestep topics.
    constants.TOPIC_CURRENT_POSE_RAW: SLAM_STREAM_QOS,
    constants.TOPIC_CURRENT_POINTS_RAW: SLAM_STREAM_QOS,
    constants.TOPIC_CURRENT_POSE_METRIC: SLAM_STREAM_QOS,
    constants.TOPIC_CURRENT_POINTS_METRIC: SLAM_STREAM_QOS,
    constants.TOPIC_OCCUPANCY_GRID: DEFAULT_QOS,
    constants.TOPIC_FRONTIER_GOALS: DEFAULT_QOS,
    constants.TOPIC_PLANNED_PATH: DEFAULT_QOS,
    constants.TOPIC_MISSION_STATUS: DEFAULT_QOS,
    constants.TOPIC_KEYFRAME_POINTS: DEFAULT_QOS,
    constants.TOPIC_TRAJECTORY: DEFAULT_QOS,
    # ToF-based scale estimation topics (middleware/telemetry_bridge.py)
    constants.TOPIC_TOF_HEIGHT_CM: DEFAULT_QOS,
    constants.TOPIC_BARO_HEIGHT_M: DEFAULT_QOS,
}


def qos_for_topic(topic_name: str) -> QoSProfile:
    """
    Return the QoS profile required for a given topic name.

    Raises KeyError (deliberately, not swallowed) if the topic isn't
    registered in _TOPIC_QOS - an unregistered topic is a bug, not a
    case to silently default around, since a wrong QoS profile is
    exactly the kind of mismatch that breaks the SLAM handshake.
    """
    if topic_name not in _TOPIC_QOS:
        raise KeyError(
            f"No QoS profile registered for topic '{topic_name}'. "
            "Add it to middleware.topic_manager._TOPIC_QOS before "
            "publishing/subscribing on it."
        )
    return _TOPIC_QOS[topic_name]


class TopicManager:
    """
    Wraps a single rclpy Node's publisher/subscriber creation so every
    call goes through the topic -> QoS mapping above instead of each
    caller picking its own QoS inline.

    ros_bridge.py should hold one instance of this (or call the
    module-level qos_for_topic() directly if it prefers to manage its
    own publisher/subscriber handles) rather than calling
    node.create_publisher()/create_subscription() itself.
    """

    def __init__(self, node):
        self._node = node
        self._publishers = {}
        self._subscriptions = {}

    def get_publisher(self, topic_name: str, msg_type):
        """
        Return the publisher for (topic_name, msg_type), creating it on
        first use. Repeated calls with the same key reuse the existing
        publisher rather than creating duplicates.
        """
        key = (topic_name, msg_type)
        if key not in self._publishers:
            qos = qos_for_topic(topic_name)
            self._publishers[key] = self._node.create_publisher(msg_type, topic_name, qos)
        return self._publishers[key]

    def get_subscription(self, topic_name: str, msg_type, callback):
        """
        Create a subscription for (topic_name, msg_type) with the
        correct QoS and keep a reference to it (rclpy subscriptions are
        garbage-collected if nothing holds a reference).

        Unlike get_publisher, this does not dedupe repeated calls on
        the same topic/type - a second call with a different callback
        is assumed to be intentional (e.g. two independent pieces of
        logic listening to the same topic). Keep your own reference to
        the returned subscription if you need it later.
        """
        qos = qos_for_topic(topic_name)
        subscription = self._node.create_subscription(msg_type, topic_name, callback, qos)
        self._subscriptions.setdefault(topic_name, []).append(subscription)
        return subscription

    def destroy_all(self):
        """
        Destroy every publisher/subscription this instance created.
        Call from the owning node's shutdown path (e.g. before
        node.destroy_node()) so nothing leaks on a clean restart.
        """
        for publisher in self._publishers.values():
            self._node.destroy_publisher(publisher)
        self._publishers.clear()

        for subscription_list in self._subscriptions.values():
            for subscription in subscription_list:
                self._node.destroy_subscription(subscription)
        self._subscriptions.clear()
