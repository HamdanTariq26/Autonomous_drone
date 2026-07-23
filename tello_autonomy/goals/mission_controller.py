"""
goals/mission_controller.py

High-level ROS2 node for managing autonomous flight states.
Subscribes to live metric poses. Uses TrajectoryTracker to compute
velocities and sends them to the drone via CommandHandler.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tello_autonomy_msgs.srv import SearchPlan
from config import constants
from goals.trajectory_tracker import TrajectoryTracker
import math
import time

class MissionState:
    IDLE = 0
    NAVIGATING = 1
    EXPLORING = 2

class MissionControllerNode(Node):
    def __init__(self, command_handler, node_name="mission_controller"):
        super().__init__(node_name)
        self._command_handler = command_handler
        self._tracker = TrajectoryTracker()
        self._state = MissionState.IDLE

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_z = 0.0
        self._current_yaw = 0.0
        self._has_pose = False
        # Track if an NbvPlan or SearchPlan request is currently pending
        self._nbv_request_in_flight = False
        self._search_request_in_flight = False

        # --- Tracking-loss recovery sweep ---
        # When SLAM loses tracking, the drone steps through 90-degree yaw
        # increments (N, E, S, W) and holds each heading long enough for
        # ORB-SLAM3 to scan for features. The instant a fresh pose arrives
        # the sweep ends and normal flight resumes.
        self._last_pose_time = None          # wall-clock time of last fresh pose
        self._tracking_lost = False          # True while we are in recovery mode
        self._TRACKING_TIMEOUT_SEC = 0.5     # declare loss after this gap
        self._RECOVERY_YAW_SPEED = 20        # yaw command (0-100) while turning
        self._RECOVERY_HOLD_SEC = 1.5        # seconds to hold each 90° heading
        self._recovery_step = 0              # which 90° step we are currently on (0-3)
        self._recovery_step_start = None     # monotonic time the current step started
        self._recovery_turning = False       # True = still rotating to next step
        self._recovery_yaw_elapsed = 0.0     # seconds we have been turning this step

        # Subscriber for live pose
        self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_METRIC,
            self._pose_callback,
            10
        )

        # Subscriber for goal pose (allows triggering missions from terminal)
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_callback,
            10
        )

        # Clients for planning services
        self._search_client = self.create_client(SearchPlan, '/tello_autonomy/search_plan')
        
        # NbvPlan client - service name matches the C++ node's registration:
        # create_service(NbvPlan, "nbvplanner") without a namespace resolves to /nbvplanner
        from tello_autonomy_msgs.srv import NbvPlan
        self._nbv_client = self.create_client(NbvPlan, '/nbvplanner')

        # Relay our PoseStamped to the format exploration_cpp expects.
        # The C++ node subscribes to "pose" (PoseWithCovarianceStamped) under
        # the root namespace -> /pose.
        self._pose_relay_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/pose',
            10
        )

        # Subscriber to trigger exploration mode
        from std_msgs.msg import Empty
        self.create_subscription(Empty, '/explore', self._explore_callback, 10)

        # Control loop timer
        timer_period = 1.0 / constants.GOALS_LOOP_RATE_HZ
        self._timer = self.create_timer(timer_period, self._control_loop)

        self.get_logger().info("MissionController initialized.")

    def _explore_callback(self, msg):
        self.get_logger().info("Triggering exploration mode...")
        self.start_exploration()

    def start_exploration(self):
        """
        Request the next best view from exploration_cpp and navigate to it.
        """
        if self._nbv_request_in_flight:
            return

        if not self._has_pose:
            self.get_logger().error("Cannot start exploration: no live pose received yet.")
            return

        from tello_autonomy_msgs.srv import NbvPlan
        if not self._nbv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("NbvPlan service not available!")
            return

        req = NbvPlan.Request()
        req.header.frame_id = "map"
        
        self.get_logger().info("Requesting Next-Best-View path...")
        self._nbv_request_in_flight = True
        future = self._nbv_client.call_async(req)
        future.add_done_callback(self._on_nbv_plan_received)

    def _on_nbv_plan_received(self, future):
        self._nbv_request_in_flight = False
        try:
            response = future.result()
            if not response.path:
                self.get_logger().info("Exploration finished or no path found.")
                self.cancel_mission()
                return

            # NbvPlan.srv response is geometry_msgs/Pose[] (NOT PoseStamped[]).
            waypoints = []
            for pose in response.path:
                q = pose.orientation
                # SLAM frame: Yaw is rotation around Y-axis
                yaw = 2.0 * math.atan2(q.y, q.w)
                
                waypoints.append({
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z,
                    'yaw': yaw
                })

            if len(waypoints) > 1:
                waypoints.pop(0)

            self._tracker.set_path(waypoints)
            self._state = MissionState.EXPLORING
            self.get_logger().info(f"Exploration path received ({len(waypoints)} waypoints). Moving.")
        except Exception as e:
            self.get_logger().error(f"NbvPlan Service call failed: {e}")
            self.cancel_mission()

    def _pose_callback(self, msg: PoseStamped):
        self._current_x = msg.pose.position.x
        self._current_y = msg.pose.position.y
        self._current_z = msg.pose.position.z

        # Extract yaw from quaternion (SLAM frame: rotation around Y-axis)
        q = msg.pose.orientation
        self._current_yaw = 2.0 * math.atan2(q.y, q.w)

        self._has_pose = True
        self._last_pose_time = time.monotonic()

        # If we were in recovery sweep mode, tracking just came back - exit immediately.
        if self._tracking_lost:
            self.get_logger().info("Tracking recovered! Resuming mission.")
            self._tracking_lost = False
            self._recovery_yaw_accumulated = 0.0

        # Relay pose to exploration_cpp which expects PoseWithCovarianceStamped.
        # Covariance is all zeros - exploration only uses the mean position.
        relay = PoseWithCovarianceStamped()
        relay.header = msg.header
        relay.header.frame_id = "map"  # Force map frame to bypass tf lookups
        relay.pose.pose = msg.pose
        # covariance is 36-element array, default zero - fine for this use
        self._pose_relay_pub.publish(relay)

    def _goal_callback(self, msg: PoseStamped):
        """
        Triggered when a user publishes to /goal_pose
        """
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_z = msg.pose.position.z
        self.get_logger().info(f"Received external goal command: {target_x}, {target_y}, {target_z}")
        self.start_mission_to_goal(target_x, target_y, target_z)

    def start_mission_to_goal(self, target_x, target_y, target_z):
        """
        Request a path from search_cpp and start navigating to it.
        """
        if self._search_request_in_flight:
            return

        if not self._has_pose:
            self.get_logger().error("Cannot start mission: no live pose received yet.")
            return

        if not self._search_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("SearchPlan service not available!")
            return

        req = SearchPlan.Request()
        req.start.header.frame_id = "map"
        req.start.pose.position.x = self._current_x
        req.start.pose.position.y = self._current_y
        req.start.pose.position.z = self._current_z
        
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x = float(target_x)
        req.goal.pose.position.y = float(target_y)
        req.goal.pose.position.z = float(target_z)

        self.get_logger().info(f"Requesting path to ({target_x}, {target_y}, {target_z})...")
        self._search_request_in_flight = True
        future = self._search_client.call_async(req)
        future.add_done_callback(self._on_search_plan_received)

    def _on_search_plan_received(self, future):
        self._search_request_in_flight = False
        try:
            response = future.result()
            if not response.success or not response.path:
                self.get_logger().error("Path planning failed or returned empty path.")
                self.cancel_mission()
                return

            # Convert PoseStamped[] path to dict format for tracker
            waypoints = []
            for pose_stamped in response.path:
                q = pose_stamped.pose.orientation
                # SLAM frame: Yaw is rotation around Y-axis
                yaw = 2.0 * math.atan2(q.y, q.w)
                
                waypoints.append({
                    'x': pose_stamped.pose.position.x,
                    'y': pose_stamped.pose.position.y,
                    'z': pose_stamped.pose.position.z,
                    'yaw': yaw
                })
            
            # The first point is usually the current position, pop it to avoid standing still
            if len(waypoints) > 1:
                waypoints.pop(0)

            self._tracker.set_path(waypoints)
            self._state = MissionState.NAVIGATING
            self.get_logger().info(f"Path received with {len(waypoints)} waypoints. Starting navigation.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.cancel_mission()

    def cancel_mission(self):
        if self._state != MissionState.IDLE:
            self.get_logger().info("Mission canceled. Hovering.")
            self._tracker.clear_path()
            self._state = MissionState.IDLE
            # Send zero velocity to stop immediately
            self._command_handler.send_rc_control(0, 0, 0, 0)

    def _control_loop(self):
        """
        Runs at GOALS_LOOP_RATE_HZ. Checks state and sends velocities.
        """
        if self._state == MissionState.IDLE:
            return

        # ---- Tracking-loss detection ----
        # If we have never received a pose yet, just hover.
        if not self._has_pose:
            self._command_handler.send_rc_control(0, 0, 0, 0)
            return

        # Check staleness of the last pose.
        now = time.monotonic()
        pose_age = now - self._last_pose_time

        if pose_age > self._TRACKING_TIMEOUT_SEC:
            # Tracking is lost. Enter (or stay in) recovery sweep mode.
            if not self._tracking_lost:
                self.get_logger().warn(
                    "SLAM tracking lost! Starting 90-degree step sweep to relocalize."
                )
                self._tracking_lost = True
                self._recovery_step = 0
                self._recovery_step_start = now
                self._recovery_turning = True
                self._recovery_yaw_elapsed = 0.0

            dt = 1.0 / constants.GOALS_LOOP_RATE_HZ

            # --- Turning phase: rotate toward the next 90-degree heading ---
            # Tello at 20% yaw ≈ ~36 deg/s → 90° takes ~2.5s
            _TURN_DURATION_SEC = 2.5
            if self._recovery_turning:
                self._recovery_yaw_elapsed += dt
                if self._recovery_yaw_elapsed >= _TURN_DURATION_SEC:
                    # Finished turning to this heading — now hold it.
                    self._recovery_turning = False
                    self._recovery_step_start = now
                    self.get_logger().info(
                        f"Recovery: reached heading step {self._recovery_step + 1}/4, "
                        f"holding for {self._RECOVERY_HOLD_SEC:.1f}s..."
                    )
                self._command_handler.send_rc_control(0, 0, 0, self._RECOVERY_YAW_SPEED)
                return

            # --- Hold phase: stay still so the camera can see features ---
            hold_elapsed = now - self._recovery_step_start
            if hold_elapsed < self._RECOVERY_HOLD_SEC:
                # Hovering — send zero so the drone holds its position.
                self._command_handler.send_rc_control(0, 0, 0, 0)
                return

            # Hold done — advance to next 90° step.
            self._recovery_step = (self._recovery_step + 1) % 4
            self._recovery_turning = True
            self._recovery_yaw_elapsed = 0.0
            self.get_logger().info(
                f"Recovery: turning to heading step {self._recovery_step + 1}/4..."
            )
            self._command_handler.send_rc_control(0, 0, 0, self._RECOVERY_YAW_SPEED)
            return

        # ---- Normal flight ----
        if self._tracker.is_done():
            if self._state == MissionState.EXPLORING:
                self.get_logger().info("Reached end of exploration path. Requesting next view...")
                # Temporarily hover while waiting for next path
                self._command_handler.send_rc_control(0, 0, 0, 0)
                self.start_exploration()
            else:
                self.get_logger().info("Mission complete! Hovering.")
                self._state = MissionState.IDLE
                self._command_handler.send_rc_control(0, 0, 0, 0)
            return

        cmd_lr, cmd_fb, cmd_ud, cmd_yaw = self._tracker.compute_velocity_commands(
            self._current_x, self._current_y, self._current_z, self._current_yaw
        )

        # Send to drone.
        # send_rc_control silently fails (returns False) if a takeoff/land is in progress,
        # which is exactly the safety behavior we want.
        self._command_handler.send_rc_control(cmd_lr, cmd_fb, cmd_ud, cmd_yaw)
