"""
middleware/telemetry_bridge.py

Bridges drone_interface.telemetry.TelemetryMonitor's ToF and barometer
readings into ROS2, so perception/tof_scale_estimator.py (and anything else
downstream) never has to touch djitellopy directly — consistent with the
rest of middleware/, whose job is exactly this kind of drone_interface ->
ROS2 topic bridging (see ros_bridge.py for the same pattern applied to
camera frames).

Publishes raw values only — NO validity filtering happens here. This
includes the SDK's ToF near-field clamp (~10 cm when the true distance is
under ~30 cm) and its out-of-range sentinel (~8192). Filtering belongs
downstream in perception/tof_scale_estimator.py, which is the one place
that needs to reason about what counts as "valid" — this node's only job is
"get the numbers out of djitellopy and onto topics."

The TelemetryMonitor instance passed in MUST have been constructed with
"tof" and "barometer" in its fields list (see scripts/main.py). If either
field is absent, that topic simply won't be published on that tick rather
than erroring — a missing field is a wiring choice, not a fault.
"""

from std_msgs.msg import Float64
from rclpy.node import Node

from config import constants
from middleware.topic_manager import TopicManager


class TelemetryBridge(Node):
    """
        Polls a TelemetryMonitor at constants.TOF_POLL_HZ and republishes
        its "tof" and "barometer" fields as Float64 topics. Never touches
        TelloDriver or djitellopy directly - only reads through the
        TelemetryMonitor, same as every other drone_interface consumer.
    """

    def __init__(self, telemetry_monitor, node_name="telemetry_bridge"):
        """
            telemetry_monitor: a drone_interface.telemetry.TelemetryMonitor
                instance, already constructed with fields=[..., "tof",
                "barometer", ...] (see scripts/main.py). If "tof" or
                "barometer" aren't in its fields list, this bridge quietly
                publishes nothing for that topic every tick rather than
                erroring - a missing field is a wiring choice, not a fault.
        """
        super().__init__(node_name)
        self._telemetry_monitor = telemetry_monitor
        self._topics = TopicManager(self)

        self._tof_pub = self._topics.get_publisher(constants.TOPIC_TOF_HEIGHT_CM, Float64)
        self._baro_pub = self._topics.get_publisher(constants.TOPIC_BARO_HEIGHT_M, Float64)

        self._timer = self.create_timer(1.0 / constants.TOF_POLL_HZ, self._publish_tick)

        self.get_logger().info(
            f"TelemetryBridge: publishing raw ToF ({constants.TOPIC_TOF_HEIGHT_CM}) and "
            f"barometer ({constants.TOPIC_BARO_HEIGHT_M}) at {constants.TOF_POLL_HZ} Hz."
        )

    # ****************************************************************************************
    def _publish_tick(self):
        """
            Polls TelemetryMonitor and publishes whatever fields are present.
            Any single missing/failed field reading (see TelemetryMonitor.read()'s
            own per-field try/except) just means that topic gets skipped this
            tick - never crashes the publish timer.
        """
        reading = self._telemetry_monitor.read()

        tof_cm = reading.get("tof")
        if tof_cm is not None:
            msg = Float64()
            msg.data = float(tof_cm)
            self._tof_pub.publish(msg)

        baro_m = reading.get("barometer")
        if baro_m is not None:
            msg = Float64()
            msg.data = float(baro_m)
            self._baro_pub.publish(msg)
    # ****************************************************************************************

    # ****************************************************************************************
    def destroy_node(self):
        if self._timer is not None:
            self._timer.cancel()
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************
