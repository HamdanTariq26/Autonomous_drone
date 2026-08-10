"""
middleware/ext_tof_bridge.py

Bridges drone_interface.ext_tof_driver.ExtTofDriver's extension-kit ToF
readings into ROS2, mirroring middleware/telemetry_bridge.py's role for
the built-in ToF/barometer. Publishes raw millimeter distance plus a
validity/staleness flag - NO scale-factor math happens here, same
separation telemetry_bridge.py keeps from tof_scale_estimator.py.

Publishes on:
  TOPIC_EXT_TOF_DISTANCE_MM  (std_msgs/Float64) - raw distance, mm.
    Only published when the driver has a value at all (parsed at
    least once); still published even when out of the valid range, so
    downstream consumers can distinguish "sensor says too far" from
    "no data yet."
  TOPIC_EXT_TOF_VALID        (std_msgs/Bool) - True only when the
    latest reading is both fresh (not stale/hung) and inside the
    sensor's valid range.

The ExtTofDriver instance passed in must already have start() called
on it (or be started by the same wiring that constructs this bridge -
see scripts/main.py) - this bridge only reads via driver.read(), it
never starts/stops the driver itself.
"""

from std_msgs.msg import Bool, Float64
from rclpy.node import Node

from config import constants
from middleware.topic_manager import TopicManager


class ExtTofBridge(Node):
    """
        Polls an ExtTofDriver at constants.EXT_TOF_BRIDGE_POLL_HZ and
        republishes its distance (mm) and validity as ROS2 topics.
        Never touches TelloDriver or djitellopy directly - only reads
        through the driver, same as every other drone_interface
        consumer in middleware/.
    """

    def __init__(self, ext_tof_driver, node_name="ext_tof_bridge"):
        """
            ext_tof_driver: a drone_interface.ext_tof_driver.ExtTofDriver
                instance, already constructed and started (see
                scripts/main.py). This bridge only calls .read() on it.
        """
        super().__init__(node_name)
        self._driver = ext_tof_driver
        self._topics = TopicManager(self)

        self._distance_pub = self._topics.get_publisher(
            constants.TOPIC_EXT_TOF_DISTANCE_MM, Float64
        )
        self._valid_pub = self._topics.get_publisher(
            constants.TOPIC_EXT_TOF_VALID, Bool
        )

        self._timer = self.create_timer(
            1.0 / constants.EXT_TOF_BRIDGE_POLL_HZ, self._publish_tick
        )

        self.get_logger().info(
            f"ExtTofBridge: publishing extension-kit ToF distance "
            f"({constants.TOPIC_EXT_TOF_DISTANCE_MM}) and validity "
            f"({constants.TOPIC_EXT_TOF_VALID}) at "
            f"{constants.EXT_TOF_BRIDGE_POLL_HZ} Hz."
        )

    # ****************************************************************************************
    def _publish_tick(self):
        reading = self._driver.read()

        if reading["distance_mm"] is not None:
            msg = Float64()
            msg.data = float(reading["distance_mm"])
            self._distance_pub.publish(msg)

        valid_msg = Bool()
        valid_msg.data = bool(reading["valid"])
        self._valid_pub.publish(valid_msg)
    # ****************************************************************************************

    # ****************************************************************************************
    def destroy_node(self):
        if self._timer is not None:
            self._timer.cancel()
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************
