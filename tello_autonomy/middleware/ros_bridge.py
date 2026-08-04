"""
middleware/ros_bridge.py

The actual ROS2 Node for this package - bridges drone_interface's
camera frames to the C++ ORB-SLAM3 node over ROS2 topics, and performs
the startup handshake required before the C++ side will start
processing frames.

Depends on:
  - middleware.topic_manager   (publisher/subscriber creation + QoS)
  - middleware.message_types   (data <-> ROS2 message conversions)
  - drone_interface.frame_receiver.FrameReceiver (frame source)

Does NOT depend on tello_driver.py or command_handler.py directly -
this node only reads frames via FrameReceiver, it never sends flight
commands. Whatever wires this node together (scripts/main.py, not yet
built) is responsible for constructing TelloDriver + FrameReceiver
first and handing the FrameReceiver in here.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

from config import constants
from middleware import message_types
from middleware.topic_manager import TopicManager

# How often to re-publish the handshake settings message while waiting
# for the C++ node's ack. Independent of DEFAULT_MAX_PUB_RATE_HZ - the
# handshake is a startup-only concern, not the steady-state frame rate.
HANDSHAKE_RETRY_INTERVAL_SEC = 1.0


class RosBridge(Node):
    """
        ROS2 node that performs the SLAM handshake, then publishes
        camera frames + timestamps on a fixed-rate timer, publishing each
        frame in-memory for downstream keyframe/depth matching.
    """

    def __init__(self, frame_receiver, node_name="tello_ros_bridge"):
        """
            frame_receiver: a drone_interface.frame_receiver.FrameReceiver
                            instance, already wired to a connected
                            TelloDriver. This node never touches
                            TelloDriver directly - it only reads frames
                            through frame_receiver.
        """
        super().__init__(node_name)

        self._frame_receiver = frame_receiver
        self._topics = TopicManager(self)

        self._frame_id = 0
        self._handshake_acked = False
        self._handshake_timer = None
        self._publish_timer = None

        # Handshake pub/sub.
        self._settings_pub = self._topics.get_publisher(
            constants.TOPIC_EXPERIMENT_SETTINGS, String
        )
        self._topics.get_subscription(
            constants.TOPIC_EXP_SETTINGS_ACK, String, self._on_ack
        )

        # Steady-state frame publishers.
        self._img_pub = self._topics.get_publisher(constants.TOPIC_IMG_MSG, Image)
        self._timestep_pub = self._topics.get_publisher(constants.TOPIC_TIMESTEP_MSG, Float64)
        self._recent_frame_pub = self._topics.get_publisher(constants.TOPIC_RECENT_FRAME, Image)

        self._start_handshake()

    # ****************************************************************************************
    def _start_handshake(self):
        """
            Publishes the experiment-settings string once immediately,
            then on a repeating timer, until _on_ack() fires.

            TRANSIENT_LOCAL QoS (set in topic_manager.HANDSHAKE_QOS)
            means even a late-discovered subscriber on the C++ side
            receives the most recent publish - this is the fix for the
            "works on 2nd ros2 run, not the 1st" discovery-race issue
            documented in the handoff doc, Section 7. The retry timer
            here is a second layer of robustness in case the C++ node
            isn't even running yet when this node starts.
        """
        self._publish_settings()
        self._handshake_timer = self.create_timer(
            HANDSHAKE_RETRY_INTERVAL_SEC, self._publish_settings
        )

    # ****************************************************************************************
    def _publish_settings(self):
        if self._handshake_acked:
            return
        msg = message_types.string_msg(constants.DEFAULT_SETTINGS_NAME)
        self._settings_pub.publish(msg)

    # ****************************************************************************************
    def _on_ack(self, msg):
        """
            Subscriber callback for TOPIC_EXP_SETTINGS_ACK. Stops the
            handshake retry timer and starts the frame-publish timer,
            exactly once - repeated/late acks after the first are
            ignored.
        """
        if self._handshake_acked:
            return
        if not message_types.is_ack(msg):
            return

        self._handshake_acked = True
        if self._handshake_timer is not None:
            self._handshake_timer.cancel()
            self._handshake_timer = None

        self.get_logger().info("SLAM handshake acked - starting frame publish loop.")
        self._publish_timer = self.create_timer(
            1.0 / constants.DEFAULT_MAX_PUB_RATE_HZ, self._publish_frame_tick
        )

    # ****************************************************************************************
    def _publish_frame_tick(self):
        """
            Timer callback: grabs the latest frame, publishes it plus a
            matching timestamp, and saves the same frame to disk under
            a filename containing the identical raw timestamp float -
            required for perception/'s keyframe-to-image matching to
            line up exactly (see message_types.current_timestamp_msg
            and handoff doc Section 7).

            Any missing piece (no frame yet, conversion failure) skips
            this tick rather than raising - a single dropped tick must
            never crash the publish timer.
        """
        frame_bgr = self._frame_receiver.get_frame_bgr()
        if frame_bgr is None:
            return

        img_msg = message_types.bgr_frame_to_image_msg(frame_bgr)
        if img_msg is None:
            return

        timestep_msg, raw_timestamp = message_types.current_timestamp_msg()

        # Stamp the image header with raw_timestamp - the same value the C++
        # SLAM node will store as kf->mTimeStamp (via the timestep Float64).
        # CvBridge leaves header.stamp=0 by default, which would cause
        # _on_recent_frame to key every frame at 0.0 in _recent_frames, making
        # _find_recent_frame unable to match any real keyframe timestamp.
        secs = int(raw_timestamp)
        img_msg.header.stamp.sec = secs
        img_msg.header.stamp.nanosec = int((raw_timestamp - secs) * 1e9)

        self._img_pub.publish(img_msg)
        self._timestep_pub.publish(timestep_msg)
        self._recent_frame_pub.publish(img_msg)

        self._frame_id += 1

    # ****************************************************************************************
    def destroy_node(self):
        """
            Cancels any live timers and cleans up every publisher/
            subscription created via TopicManager before deferring to
            Node.destroy_node().
        """
        if self._handshake_timer is not None:
            self._handshake_timer.cancel()
        if self._publish_timer is not None:
            self._publish_timer.cancel()
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************