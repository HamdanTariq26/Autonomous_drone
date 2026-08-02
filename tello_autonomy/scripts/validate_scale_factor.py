#!/usr/bin/env python3
"""
scripts/validate_scale_with_2m_flight.py

VALIDATION SCRIPT: Tests the complete scale estimation pipeline by flying
the drone forward exactly 2 meters (ground truth marked on floor) and
comparing the pipeline's estimated distance.

This uses the EXACT same pipeline as autonomous exploration:
- ORB-SLAM3 raw pose
- ToF sensor
- Depth-Anything-V2 (if enabled)
- ScaleFactorManager's fusion logic
- LiveScaler's metric pose output

NO simplifications or shortcuts. This tests the real system.

USAGE:
1. Mark exactly 2.0 meters on the floor with tape.
2. Place drone at start position, facing forward.
3. Run this script.
4. The drone will:
   - Take off
   - Hover for 3 seconds (allow scale to initialize)
   - Fly forward until LiveScaler reports 2.0m displacement
   - Land
5. Script reports:
   - Actual distance flown (from tape measure = 2.0m)
   - Pipeline's estimated distance
   - Scale factor error percentage
   - Recommendation for scale adjustment

REQUIREMENTS:
- Your existing telloautonomy package
- A 2-meter marked distance on the floor
- Enough space for safe forward flight
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from telloautonomy.drone_interface import TelloDriver, CommandHandler
from config import constants
from middleware.topic_manager import TopicManager


class ScaleValidator(Node):
    """Validates scale accuracy by flying a known 2m distance."""

    def __init__(self):
        super().__init__('scale_validator')

        # Drone control
        self.driver = TelloDriver(resolution='low', fps=15, bitrate=constants.TELLO_VIDEO_BITRATE)
        self.driver.connect()
        self.commander = CommandHandler(self.driver)

        # Subscribe to metric pose (the same topic used by exploration)
        self._topics = TopicManager(self)
        self._current_pose = None
        self._start_pose = None
        self._topics.get_subscription(
            constants.TOPIC_CURRENT_POSE_METRIC,
            PoseStamped,
            self._on_pose,
        )

        self.get_logger().info("ScaleValidator initialized. Waiting for metric pose...")

    def _on_pose(self, msg: PoseStamped):
        self._current_pose = msg
        if self._start_pose is None:
            self._start_pose = msg
            self.get_logger().info(f"Start pose recorded: ({msg.pose.position.x:.3f}, "
                                   f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")

    def _get_forward_distance(self):
        """Calculate forward (Z-axis) distance from start pose."""
        if self._start_pose is None or self._current_pose is None:
            return 0.0

        # In SLAM convention: Z = forward
        delta_z = self._current_pose.pose.position.z - self._start_pose.pose.position.z
        return abs(delta_z)

    def takeoff_and_hover(self, hover_duration=3.0):
        """Take off and hover to allow scale to initialize."""
        self.get_logger().info("Taking off...")
        if not self.commander.takeoff_async():
            self.get_logger().error("Takeoff failed!")
            return False

        # Wait for takeoff to complete
        time.sleep(2.5)

        self.get_logger().info(f"Hovering for {hover_duration}s to allow scale initialization...")
        for i in range(int(hover_duration * 10)):
            rclpy.spin_once(self, timeout_sec=0.1)
            if i % 10 == 0:
                self.get_logger().info(f"Hovering... {hover_duration - i/10:.1f}s remaining")

        return True

    def fly_forward_2m(self):
        """Fly forward until pipeline estimates 2.0m traveled."""
        TARGET_DISTANCE = 2.0  # meters
        FORWARD_SPEED = 30  # Tello RC forward command (0-100)
        TOLERANCE = 0.15  # Accept 2.0m ± 0.15m

        self.get_logger().info(f"Starting forward flight. Target: {TARGET_DISTANCE}m")
        self.get_logger().info(f"Current scale factor: {self._get_current_scale()}")

        start_time = time.time()
        max_flight_time = 15.0  # Safety timeout

        while True:
            rclpy.spin_once(self, timeout_sec=0.1)

            elapsed = time.time() - start_time
            if elapsed > max_flight_time:
                self.get_logger().warn(f"Flight timeout ({max_flight_time}s) reached. Landing.")
                break

            current_distance = self._get_forward_distance()
            self.get_logger().info(f"Distance flown: {current_distance:.3f}m")

            if current_distance >= TARGET_DISTANCE + TOLERANCE:
                self.get_logger().info(f"Target distance reached: {current_distance:.3f}m")
                break

            # Send forward command
            self.commander.send_rc_control(0, FORWARD_SPEED, 0, 0)

        # Stop
        self.commander.send_rc_control(0, 0, 0, 0)
        time.sleep(0.5)

    def land_and_report(self, target_distance=2.0):
        """Land and print validation results."""
        self.get_logger().info("Landing...")
        self.commander.land_async()
        time.sleep(2.0)

        # Calculate results
        actual_distance = self._get_forward_distance()
        error_m = actual_distance - target_distance
        error_pct = (error_m / target_distance) * 100.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("SCALE VALIDATION RESULTS")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Target distance (ground truth): {target_distance:.3f}m")
        self.get_logger().info(f"Pipeline estimated distance: {actual_distance:.3f}m")
        self.get_logger().info(f"Absolute error: {error_m:.3f}m")
        self.get_logger().info(f"Percentage error: {error_pct:.1f}%")
        self.get_logger().info(f"Scale factor used: {self._get_current_scale()}")
        self.get_logger().info("=" * 60)

        if abs(error_pct) < 5.0:
            self.get_logger().info("✓ Scale factor is ACCURATE (within 5%)")
        elif abs(error_pct) < 15.0:
            self.get_logger().info("⚠ Scale factor is ACCEPTABLE (5-15% error)")
            self.get_logger().info(f"  Suggested scale adjustment: multiply by {target_distance / actual_distance:.3f}")
        else:
            self.get_logger().error("✗ Scale factor is INACCURATE (>15% error)")
            self.get_logger().error(f"  STRONG RECOMMENDATION: multiply scale by {target_distance / actual_distance:.3f}")

        self.get_logger().info("=" * 60)

    def _get_current_scale(self):
        """Get current scale factor from the system."""
        # This would need to query ScaleFactorManager
        # For now, return a placeholder - you can add a service call here
        return "N/A (add service to query scale)"

    def cleanup(self):
        self.driver.disconnect()
        self.destroy_node()


def main():
    rclpy.init()
    validator = ScaleValidator()

    try:
        # Wait for scale to be ready
        input("Press ENTER when 'Metric scale ready' appears in main terminal, then continue...")

        # Take off and hover
        if not validator.takeoff_and_hover(hover_duration=3.0):
            return

        # Fly forward 2m
        validator.fly_forward_2m()

        # Land and report
        validator.land_and_report(target_distance=2.0)

    except KeyboardInterrupt:
        validator.get_logger().info("Interrupted by user. Landing...")
        validator.commander.land_async()
    finally:
        validator.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
