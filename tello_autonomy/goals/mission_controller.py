"""
goals/mission_controller.py

High-level ROS2 node for managing autonomous flight states.
Subscribes to live metric poses. Uses TrajectoryTracker to compute
velocities and sends them to the drone via CommandHandler.

Map re-init handling (pipeline_audit.md, Finding #1):
    Every pose message's header.frame_id carries the producing map_id as
    "slam_map_<N>" (see config.constants.SLAM_MAP_FRAME_ID_PREFIX). When
    ORB-SLAM3 loses and regains tracking, it starts a brand new map_id
    whose origin is wherever the drone physically was at that moment -
    NOT the same physical point as the previous map's origin. This node
    previously had no notion of that at all: _broadcast_identity_frame()
    publishes a zero-offset "map -> slam_map_N" transform for every map
    it ever sees, and _pose_callback() just kept computing velocity
    commands against whatever pose arrived, regardless of which map
    produced it.

    occupancy_map_cpp now resets its OcTree whenever it sees the same
    map_id change (see occupancy_map_node.cpp) - a partially remapped
    room instead of a corrupted one. This node needs to cooperate with
    that: continuing to fly an in-progress mission (a path of waypoints
    computed against the OLD map's now-discarded occupancy data) across
    a map_id change would send the drone toward coordinates that no
    longer mean what they meant when the path was planned. So
    _pose_callback() now detects a map_id change explicitly (distinct
    from an ordinary tracking-loss-and-recovery on the SAME map, which
    is unaffected) and cancels any in-flight mission, exactly the way an
    explicit SearchPlan/NbvPlan failure already does elsewhere in this
    file. It is deliberately NOT attempting to reconcile old- and
    new-map coordinates via a relative transform - see the header
    comment on occupancy_map_node.hpp for why that's the harder, riskier
    option and reset-on-reinit was chosen instead.

    _broadcast_identity_frame() is UNCHANGED and still an approximation:
    it satisfies TF2 lookups (e.g. exploration_cpp's tf_buffer_ calls)
    by pretending every new map starts at the same physical origin as
    "map", which is only exactly true if the drone never moved between
    losing tracking and re-initializing. In practice it will have
    drifted. Fixing that properly needs an actual metric anchor (e.g.
    mission pads, per pipeline_audit.md Finding #8) - out of scope here.
    What this file's map_id tracking DOES fix is the more urgent half of
    the bug: making sure the mission controller stops trusting stale
    waypoints the instant it knows its coordinate frame just changed,
    rather than silently flying them.
"""

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tello_autonomy_msgs.srv import SearchPlan
from config import constants
from goals.trajectory_tracker import TrajectoryTracker
import math
import time

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

def _parse_map_id_from_frame_id(frame_id):
    """
    Reverses the "slam_map_<N>" encoding used on the C++ SLAM node
    side (config.constants.SLAM_MAP_FRAME_ID_PREFIX). Returns the
    map_id as an int, or None if frame_id doesn't match the expected
    prefix/format. Mirrors the identical helper already used in
    perception/live_scaler.py and perception/scale_factor_manager.py -
    kept as a free function here rather than importing across layers,
    since goals/ has no existing dependency on perception/.
    """
    if not frame_id.startswith(constants.SLAM_MAP_FRAME_ID_PREFIX):
        return None
    suffix = frame_id[len(constants.SLAM_MAP_FRAME_ID_PREFIX):]
    try:
        return int(suffix)
    except ValueError:
        return None


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
        
        #To fix frame_id mismatch between rivz2 and in coming messages
        
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._known_slam_frames = set()
        self._broadcast_identity_frame("slam_map_0")  # publish immediately, don't wait for a pose

        # --- Map re-init tracking (pipeline_audit.md, Finding #1) ---
        # None means "no pose received yet" - the very first pose's
        # map_id is adopted without being treated as a "change" (there is
        # nothing to cancel a mission against yet).
        self._current_map_id = None

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
                # FIX Bug3: simulator encodes yaw around Z axis (q.z, q.w).
                # ORB-SLAM3 uses Y-axis (q.y, q.w) — update here if switching to real drone.
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
        # To fix frame id mismatch between rviz and exploration message header
        frame_id = msg.header.frame_id
        if frame_id.startswith("slam_map_"):
            self._broadcast_identity_frame(frame_id)

        # --- Map re-init detection (pipeline_audit.md, Finding #1) ---
        # A genuine map_id change means SLAM lost and regained tracking
        # since the last pose - the new map's origin is NOT the same
        # physical point as the old map's origin (see module docstring).
        # Distinct from the tracking-loss recovery sweep below: that
        # handles the GAP in poses while SLAM is relocalizing on what
        # turns out to be the SAME map most of the time; this handles the
        # case where it comes back with a NEW map_id instead. The two are
        # independent and can both fire around the same event (a gap,
        # then a resumed pose that turns out to carry a new map_id).
        incoming_map_id = _parse_map_id_from_frame_id(frame_id)
        if incoming_map_id is not None:
            if self._current_map_id is not None and incoming_map_id != self._current_map_id:
                self.get_logger().warn(
                    f"SLAM map re-initialized: map_id {self._current_map_id} -> "
                    f"{incoming_map_id}. Canceling any in-flight mission - its "
                    f"waypoints were computed against the old map's occupancy "
                    f"data, which occupancy_map_cpp has now discarded."
                )
                self.cancel_mission()
            self._current_map_id = incoming_map_id

        self._current_x = msg.pose.position.x
        self._current_y = msg.pose.position.y
        self._current_z = msg.pose.position.z

        # FIX Bug3: simulator encodes yaw around Z axis (q.z, q.w).
        # ORB-SLAM3 uses Y-axis (q.y, q.w) — update here if switching to real drone.
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
                # FIX Bug3: simulator encodes yaw around Z axis (q.z, q.w).
                # ORB-SLAM3 uses Y-axis (q.y, q.w) — update here if switching to real drone.
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

    def _broadcast_identity_frame(self, frame_id: str):
        """
        Publish a static transform from map -> frame_id with zero offset.
        This satisfies TF2 checks like exploring_cpp's if the first pose hasn't arrived yet.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = frame_id
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)
