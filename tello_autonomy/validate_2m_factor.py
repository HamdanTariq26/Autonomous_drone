#!/usr/bin/env python3
"""
scripts/validate_scale_with_2m_flight.py

VALIDATION SCRIPT: Measures scale accuracy by comparing pipeline's estimated
distance vs your MANUAL 2m flight (ground truth marked on floor).

HOW IT WORKS:
1. You manually fly the drone exactly 2.0m forward (using floor tape)
2. This script records the start and end metric poses from the pipeline
3. Script calculates: what distance did the pipeline THINK we flew?
4. Script reports scale error

THIS SCRIPT DOES NOT CONTROL THE DRONE. You fly it manually.

USAGE:
1. Start run_real_drone.sh and wait for "Metric scale ready"
2. Mark exactly 2.0 meters on the floor with tape
3. Place drone at START position, facing forward
4. Run this script
5. When prompted: "Press ENTER when ready to start measurement"
6. Script records START pose
7. YOU manually fly the drone forward exactly 2.0m (use your floor tape!)
8. When drone is steady at END position: press ENTER
9. Script records END pose
10. Script reports:
    - Ground truth: 2.0m (what you measured)
    - Pipeline estimate: X.XXm (what SLAM+scale thinks)
    - Error percentage
    - Suggested scale correction factor
"""

import math
import os
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Add telloautonomy to path
THISDIR = os.path.dirname(os.path.abspath(__file__))
TELLO_AUTONOMY_ROOT = os.path.dirname(THISDIR)
if TELLO_AUTONOMY_ROOT not in sys.path:
    sys.path.insert(0, TELLO_AUTONOMY_ROOT)

from config import constants


class ScaleValidator(Node):
    """Measures scale accuracy by comparing pipeline estimate vs manual 2m flight."""

    def __init__(self):
        super().__init__('scale_validator')

        # Subscribe to metric pose (from existing main system)
        self._current_pose = None
        self._pose_sub = self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_METRIC,
            self._on_pose,
            10
        )

        self.get_logger().info("ScaleValidator initialized. Waiting for metric pose...")

    def _on_pose(self, msg: PoseStamped):
        self._current_pose = msg

    def _get_forward_distance_from_start(self, start_pose, end_pose):
        """Calculate forward (Z-axis) distance between two poses."""
        if start_pose is None or end_pose is None:
            return 0.0

        # In SLAM convention: Z = forward
        delta_z = end_pose.pose.position.z - start_pose.pose.position.z
        return abs(delta_z)

    def wait_for_pose(self, timeout_sec=10.0):
        """Wait for metric pose to become available."""
        self.get_logger().info("Waiting for metric pose...")
        start = time.time()
        while self._current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.get_logger().error("Timeout waiting for metric pose!")
                return False
        self.get_logger().info(f"Metric pose received: ({self._current_pose.pose.position.x:.3f}, "
                               f"{self._current_pose.pose.position.y:.3f}, "
                               f"{self._current_pose.pose.position.z:.3f})")
        return True

    def record_pose(self, label="Pose"):
        """Record current pose."""
        pose = self._current_pose
        if pose is None:
            self.get_logger().error(f"Cannot record {label} - no pose available!")
            return None
        
        self.get_logger().info(f"{label} recorded: ({pose.pose.position.x:.3f}, "
                               f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})")
        return pose


def main():
    rclpy.init()
    validator = ScaleValidator()

    try:
        # Wait for pose
        if not validator.wait_for_pose(timeout_sec=10.0):
            validator.destroy_node()
            rclpy.shutdown()
            return

        print("\n" + "=" * 60)
        print("SCALE VALIDATION - MANUAL FLIGHT TEST")
        print("=" * 60)
        print("\nINSTRUCTIONS:")
        print("1. Make sure drone is at START position (beginning of your 2m tape)")
        print("2. Drone should be facing forward (along the 2m line)")
        print("3. When ready, press ENTER to record START pose")
        print("4. Manually fly drone forward exactly 2.0m (use your floor tape)")
        print("5. Hold drone steady at END position")
        print("6. Press ENTER to record END pose")
        print("=" * 60)

        input("\nPress ENTER when drone is at START position and ready...")

        # Record start pose
        start_pose = validator.record_pose("START pose")
        if start_pose is None:
            validator.destroy_node()
            rclpy.shutdown()
            return

        print("\n" + "-" * 60)
        print("NOW FLY THE DRONE:")
        print("- Manually fly forward exactly 2.0 meters (use your floor tape)")
        print("- Keep drone facing forward (don't rotate)")
        print("- Fly straight along the tape line")
        print("- When steady at 2.0m mark, press ENTER below")
        print("-" * 60)

        input("\nPress ENTER when drone is steady at END position (2.0m mark)...")

        # Record end pose
        end_pose = validator.record_pose("END pose")
        if end_pose is None:
            validator.destroy_node()
            rclpy.shutdown()
            return

        # Calculate pipeline's estimated distance
        pipeline_distance = validator._get_forward_distance_from_start(start_pose, end_pose)

        # Ground truth (you measured this)
        ground_truth = 2.0  # meters

        # Calculate error
        error_m = pipeline_distance - ground_truth
        error_pct = (error_m / ground_truth) * 100.0

        # Calculate correction factor
        if pipeline_distance > 0:
            correction_factor = ground_truth / pipeline_distance
        else:
            correction_factor = 1.0

        # Report results
        print("\n" + "=" * 60)
        print("SCALE VALIDATION RESULTS")
        print("=" * 60)
        print(f"Ground truth distance (your measurement): {ground_truth:.3f}m")
        print(f"Pipeline estimated distance:              {pipeline_distance:.3f}m")
        print(f"Absolute error:                           {error_m:+.3f}m")
        print(f"Percentage error:                         {error_pct:+.1f}%")
        print("=" * 60)

        if abs(error_pct) < 5.0:
            print("✓ Scale factor is ACCURATE (within 5%)")
            print(f"  Current scale is GOOD - no adjustment needed")
        elif abs(error_pct) < 15.0:
            print("⚠ Scale factor is ACCEPTABLE (5-15% error)")
            print(f"  Suggested correction: multiply scale by {correction_factor:.3f}")
            print(f"  Example: if current scale is 4.65, new scale = {4.65 * correction_factor:.3f}")
        else:
            print("✗ Scale factor is INACCURATE (>15% error)")
            print(f"  STRONG RECOMMENDATION: multiply scale by {correction_factor:.3f}")
            print(f"  Example: if current scale is 4.65, new scale = {4.65 * correction_factor:.3f}")

        print("=" * 60)

        # Additional info
        print("\nNEXT STEPS:")
        print("- If error > 15%, adjust your scale factor in the code")
        print("- Look for: SCALE_RATIO_MIN, SCALE_RATIO_MAX, or similar constants")
        print("- Multiply by the correction factor above")
        print("- Re-run this test to verify improvement")
        print("- Run 3-5 times and average the results for better accuracy")
        print("=" * 60 + "\n")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
