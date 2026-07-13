"""
middleware/message_types.py

Pure conversion functions between plain Python/numpy data and ROS2
message types. No Node, no publishers/subscribers, no I/O - every
function here takes plain data in and hands a ROS2 message (or plain
data) back out. This means these conversions can be unit-tested without
rclpy.init() ever being called, and reused by any node that needs them
without inheriting ros_bridge.py's handshake/timer logic.
"""

import time

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, String

_br = CvBridge()


def bgr_frame_to_image_msg(frame_bgr):
    """
        Converts a BGR numpy array (as returned by
        drone_interface.frame_receiver.FrameReceiver.get_frame_bgr())
        into a sensor_msgs/Image message.

        Returns None if the conversion fails (logged, not raised) -
        callers publishing on a timer should treat None as "skip this
        tick" rather than crash a publish loop over one bad frame.
    """
    try:
        return _br.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
    except CvBridgeError as e:
        print(f"bgr_frame_to_image_msg conversion failed: {e}")
        return None


def current_timestamp_msg():
    """
        Returns (Float64 message wrapping time.time(), the raw float
        value used). The raw value is also returned because callers
        that save frame images to disk (see drone_interface /
        middleware wiring) need the exact same numeric value used in
        the filename, so keyframe-matching-by-timestamp later lines up
        exactly - returning both avoids two separate time.time() calls
        drifting apart by microseconds.
    """
    now = time.time()
    msg = Float64()
    msg.data = now
    return msg, now


def string_msg(text):
    msg = String()
    msg.data = text
    return msg


def is_ack(msg, expected="ACK"):
    """True if a received std_msgs/String message's data matches expected."""
    return msg.data == expected


def float_list_msg(values):
    """Wraps a plain list/iterable of floats into a Float64MultiArray message."""
    msg = Float64MultiArray()
    msg.data = list(values)
    return msg


def float_list_from_msg(msg):
    """Unwraps a Float64MultiArray message back into a plain Python list."""
    return list(msg.data)