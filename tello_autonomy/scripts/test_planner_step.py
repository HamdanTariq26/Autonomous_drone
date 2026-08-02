#!/usr/bin/env python3
"""
scripts/test_planner_step.py

Step-by-step diagnostic and test script for NBV Planner:
1. Checks topic availability: current pose, occupancy grid, and nbvplanner service.
2. Sends a service request to 'nbvplanner'.
3. Logs and displays the generated waypoints (Position XYZ, Orientation/Yaw).
4. Computes distance and vector to next waypoint.
5. Pauses for interactive user verification before repeating.
"""

import os
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from octomap_msgs.msg import Octomap
from tello_autonomy_msgs.srv import NbvPlan
from nav_msgs.msg import Odometry

_TELLO_AUTONOMY_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TELLO_AUTONOMY_ROOT not in sys.path:
    sys.path.insert(0, _TELLO_AUTONOMY_ROOT)

from config import constants


class PlannerStepTester(Node):
    def __init__(self):
        super().__init__('planner_step_tester')

        self._current_pose = None
        self._octomap_received = False

        # Subscriptions to monitor readiness
        self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_METRIC,
            self._pose_callback,
            10
        )
        self.create_subscription(
            Octomap,
            constants.TOPIC_OCCUPANCY_GRID,
            self._octomap_callback,
            10
        )

        # Publisher to feed pose to planner if planner subscribes to 'pose'
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "pose",
            10
        )

        # Service client for NBV planner
        self._client = self.create_client(NbvPlan, "nbvplanner")

        self.get_logger().info("Planner Step Tester node initialized.")

    def _pose_callback(self, msg: PoseStamped):
        self._current_pose = msg.pose
        # Forward to 'pose' topic expected by nbv_planner_node
        pwcs = PoseWithCovarianceStamped()
        pwcs.header = msg.header
        pwcs.pose.pose = msg.pose
        self._pose_pub.publish(pwcs)

    def _octomap_callback(self, msg: Octomap):
        self._octomap_received = True

    def check_readiness(self):
        ready = True
        print("\n--- NBV PLANNER READINESS CHECK ---")
        if self._current_pose is not None:
            p = self._current_pose.position
            print(f"[OK] Current Metric Pose: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m")
        else:
            print("[FAIL] Current Metric Pose: NOT RECEIVED on " + constants.TOPIC_CURRENT_POSE_METRIC)
            ready = False

        if self._octomap_received:
            print("[OK] Occupancy Grid: RECEIVED on " + constants.TOPIC_OCCUPANCY_GRID)
        else:
            print("[FAIL] Occupancy Grid: NOT RECEIVED on " + constants.TOPIC_OCCUPANCY_GRID)
            ready = False

        srv_ok = self._client.wait_for_service(timeout_sec=1.0)
        if srv_ok:
            print("[OK] Service 'nbvplanner': AVAILABLE")
        else:
            print("[FAIL] Service 'nbvplanner': NOT AVAILABLE (Is nbv_planner_node running?)")
            ready = False

        print("------------------------------------\n")
        return ready

    def request_plan_step(self):
        if not self._client.wait_for_service(timeout_sec=2.0):
            print("ERROR: Service 'nbvplanner' not available.")
            return False

        req = NbvPlan.Request()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = "map"

        print("Sending request to 'nbvplanner' service...")
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            res = future.result()
            path = res.path
            print(f"\n=======================================================")
            print(f"         NBV PLANNER STEP RESPONSE RESULT              ")
            print(f"=======================================================")
            print(f" Total Waypoints Received: {len(path)}")

            if len(path) == 0:
                print(" WARNING: Planner returned an EMPTY path! (No gain found or map empty)")
            else:
                for idx, wp in enumerate(path):
                    p = wp.position
                    # Compute yaw from quaternion
                    q = wp.orientation
                    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                    yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                    dist_from_curr = 0.0
                    if self._current_pose:
                        cp = self._current_pose.position
                        dist_from_curr = math.sqrt((p.x - cp.x)**2 + (p.y - cp.y)**2 + (p.z - cp.z)**2)

                    print(f"  Waypoint #{idx+1}: Pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m | Yaw={yaw_deg:.1f}° | DistFromCurrent={dist_from_curr:.2f} m")
            print(f"=======================================================\n")
            return True
        else:
            print("ERROR: Service call to 'nbvplanner' failed or timed out!")
            return False


def main(args=None):
    rclpy.init(args=args)
    tester = PlannerStepTester()

    # Spin briefly in background thread or step-loop
    print("Waiting 2 seconds to gather topic status...")
    start = time.time()
    while time.time() - start < 2.0:
        rclpy.spin_once(tester, timeout_sec=0.1)

    while True:
        ready = tester.check_readiness()
        if not ready:
            print("System not ready yet. Make sure main.py, SLAM, occupancy_map_node, and nbv_planner_node are running.")

        cmd = input("Press [ENTER] to trigger single NBV planning step (or 'q' to quit): ")
        if cmd.strip().lower() == 'q':
            break

        # Spin to refresh topics
        for _ in range(5):
            rclpy.spin_once(tester, timeout_sec=0.1)

        tester.request_plan_step()

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
