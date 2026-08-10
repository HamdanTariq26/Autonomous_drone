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

Recalibration hop (MissionState.RECALIBRATING):
    After a SLAM map re-init, ToFScaleEstimator needs at least
    MIN_DELTA_TOF_M (20 cm) of vertical motion before it can accumulate
    the samples required for reliable=True. If the drone is hovering
    stationary post-reinit, that motion never happens passively, so the
    system would otherwise wait up to MAX_ALIGNMENT_GAP_SEC (25 s) for
    a Depth-Anything result instead.

    _start_reinit_calibration_hop() manufactures the needed motion:
      Phase 'escape_near_field': climb until ToF >= TARGET_MARGIN_CM (45 cm)
        - ensures we are above the near-field clamp zone.
      Phase 'climb_delta': continue climbing until delta_tof from the
        baseline >= DELTA_TARGET_CM (30 cm) - crosses MIN_DELTA_TOF_M.
      Phase 'wait_scale': hover; poll scale_factor_manager for the new
        map_id's scale. Finish on success or WAIT_SCALE_TIMEOUT_SEC.
    A hard HOP_TIMEOUT_SEC = 8 s cap terminates the whole maneuver
    regardless of phase - falls through to _finish_recalibration() which
    transitions to IDLE and calls start_exploration().

    RC ownership: is_active() returns True during RECALIBRATING (it
    is non-IDLE), so ManualControl's idle-hover loop stays suppressed
    exactly as it does during NAVIGATING/EXPLORING - no extra logic.
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
    RECALIBRATING = 3   # deliberate calibration hop after map re-init

class MissionControllerNode(Node):
    def __init__(self, command_handler, scale_factor_manager=None, node_name="mission_controller"):
        super().__init__(node_name)
        self._command_handler = command_handler
        self._tracker = TrajectoryTracker()
        self._state = MissionState.IDLE

        # Reference to ScaleFactorManager (same process) so _handle_recalibration
        # can poll whether the new map's scale has landed yet.
        self._scale_factor_manager = scale_factor_manager

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_z = 0.0
        self._current_yaw = 0.0
        self._has_pose = False
        # Track if an NbvPlan or SearchPlan request is currently pending
        self._nbv_request_in_flight = False
        self._search_request_in_flight = False
        self._last_auto_cmd_time = None

        # --- Recalibration hop state (MissionState.RECALIBRATING) ---
        self._latest_tof_cm = None          # updated by _on_tof subscription below
        self._recal_map_id = None           # new map_id the hop is calibrating for
        self._recal_phase = None            # 'escape_near_field' | 'climb_delta' | 'wait_scale'
        self._recal_start_time = None       # monotonic time the hop began
        self._recal_baseline_tof_cm = None  # ToF reading at start of climb_delta phase
        self._recal_wait_start = None       # monotonic time 'wait_scale' phase began

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
        # When SLAM loses tracking the drone rotates in small steps, dwelling
        # at each heading long enough for ORB-SLAM3 to attempt relocalization
        # or map re-initialization. Recovery is detected passively: if a fresh
        # pose arrives during the dwell, _pose_callback clears _tracking_lost
        # and the pose_age check at the top of _control_loop exits the branch
        # naturally - no separate polling logic needed.
        self._last_pose_time = None          # wall-clock time of last fresh pose
        self._tracking_lost = False          # True while we are in recovery mode
        self._TRACKING_TIMEOUT_SEC = 0.5     # declare loss after this gap
        self._RECOVERY_YAW_SPEED = 20        # yaw command (0-100) while turning
        self._RECOVERY_YAW_STEP_DEG = 35     # smaller step than old 90° — finer search
        self._RECOVERY_STEP_HOLD_SEC = 5.0   # dwell time to let SLAM attempt reloc/init
        self._recovery_map_id_at_loss = None         # map_id active when loss was declared
        self._recovery_step_start = None             # monotonic time the current step started
        self._recovery_turning = False               # True = currently rotating to next step
        self._recovery_yaw_elapsed = 0.0             # seconds spent turning this step

        # Subscriber for raw (unscaled) SLAM pose - used ONLY for map_id
        # change detection. Raw poses arrive before any scale factor exists
        # for the new map, so this lets the calibration hop start immediately
        # at re-init time rather than after the alignment deadline expires.
        self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_RAW,
            self._on_raw_pose,
            10
        )

        # Subscriber for metric (scaled) pose - position tracking + relay.
        # Map_id change detection has moved to _on_raw_pose above.
        self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_METRIC,
            self._pose_callback,
            10
        )

        # Subscriber for raw ToF height - used by _handle_recalibration to
        # track altitude during the calibration hop.
        from std_msgs.msg import Float64
        self.create_subscription(
            Float64,
            constants.TOPIC_TOF_HEIGHT_CM,
            self._on_tof,
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

    def is_active(self):
        return (
            self._state != MissionState.IDLE
            or self._nbv_request_in_flight
            or self._search_request_in_flight
        )

    def time_since_last_cmd(self):
        if self._last_auto_cmd_time is None:
            return float('inf')
        return time.monotonic() - self._last_auto_cmd_time

    def _send_rc_control(self, lr, fb, ud, yaw):
        self._last_auto_cmd_time = time.monotonic()
        return self._command_handler.send_rc_control(lr, fb, ud, yaw)

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
        self._state = MissionState.EXPLORING
        self._last_auto_cmd_time = time.monotonic()
        future = self._nbv_client.call_async(req)
        future.add_done_callback(self._on_nbv_plan_received)

    def _on_nbv_plan_received(self, future):
        self._nbv_request_in_flight = False
        if self._state == MissionState.IDLE:
            self.get_logger().info("NBV plan received, but mission was canceled by manual override — discarding path.")
            return

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

            self._tracker.set_path(waypoints, require_final_yaw_lock=False)  # exploration: skip final yaw-lock
            self._state = MissionState.EXPLORING
            self.get_logger().info(f"Exploration path received ({len(waypoints)} waypoints). Moving.")
        except Exception as e:
            self.get_logger().error(f"NbvPlan Service call failed: {e}")
            self.cancel_mission()

    def _on_raw_pose(self, msg: PoseStamped):
        """
        Subscribes to TOPIC_CURRENT_POSE_RAW for map_id change detection.

        Raw poses arrive from the C++ SLAM node before any scale factor
        exists for the new map_id, and therefore before live_scaler starts
        publishing metric poses for it. Detecting the transition here means
        the calibration hop starts immediately at re-init time - inside the
        25 s alignment window - rather than 13+ seconds after it expires
        (which was the failure mode when detection lived in _pose_callback,
        a metric-topic subscriber).

        Position state (x/y/z/yaw) is NOT updated here - raw poses are in
        arbitrary SLAM units, not meters. That stays in _pose_callback.
        """
        frame_id = msg.header.frame_id
        # Broadcast identity TF frame as soon as we see a new slam_map_N id,
        # satisfying TF2 lookups before the first metric pose arrives.
        if frame_id.startswith(constants.SLAM_MAP_FRAME_ID_PREFIX):
            self._broadcast_identity_frame(frame_id)

        incoming_map_id = _parse_map_id_from_frame_id(frame_id)
        if incoming_map_id is None:
            return

        if self._current_map_id is not None and incoming_map_id != self._current_map_id:
            self.get_logger().warn(
                f"SLAM map re-initialized: map_id {self._current_map_id} -> "
                f"{incoming_map_id}. Starting calibration hop immediately "
                f"(detected on raw pose stream, before scale/metric poses)."
            )
            self.cancel_mission()
            self._start_reinit_calibration_hop(incoming_map_id)
            # If tracking_lost was active, clear it now. The hop handles
            # exploration resumption; the metric-pose tracking_lost block in
            # _pose_callback must not independently call start_exploration.
            if self._tracking_lost:
                self._tracking_lost = False
                self._recovery_map_id_at_loss = None

        self._current_map_id = incoming_map_id

    def _pose_callback(self, msg: PoseStamped):
        """
        Handles TOPIC_CURRENT_POSE_METRIC: updates position state, handles
        same-map tracking-loss recovery, and relays pose to exploration_cpp.

        Map_id change detection has moved to _on_raw_pose (raw pose stream)
        so the calibration hop fires before scale/metric poses exist for the
        new map. This callback only sees a new map_id after scale is ready
        - by then the hop is already running or completed.
        """
        self._current_x = msg.pose.position.x
        self._current_y = msg.pose.position.y
        self._current_z = msg.pose.position.z

        # FIX Bug3: simulator encodes yaw around Z axis (q.z, q.w).
        # ORB-SLAM3 uses Y-axis (q.y, q.w) — update here if switching to real drone.
        q = msg.pose.orientation
        self._current_yaw = 2.0 * math.atan2(q.y, q.w)

        self._has_pose = True
        self._last_pose_time = time.monotonic()

        # Tracking-loss recovery: if SLAM came back on the SAME map, resume
        # exploration immediately. If it came back on a NEW map, _on_raw_pose
        # already cleared _tracking_lost and started the calibration hop when
        # raw poses arrived - so the else branch here is a safety fallback only.
        if self._tracking_lost:
            incoming_map_id = _parse_map_id_from_frame_id(msg.header.frame_id)
            if incoming_map_id == self._recovery_map_id_at_loss:
                was_active = (self._state != MissionState.IDLE)
                self._tracking_lost = False
                self._recovery_map_id_at_loss = None
                if was_active:
                    self.get_logger().info(
                        f"Re-localized on same map (map_id={incoming_map_id}). "
                        "Auto-resuming exploration."
                    )
                    self.start_exploration()
                else:
                    self.get_logger().info(
                        f"Re-localized on same map (map_id={incoming_map_id}), "
                        "but mission was canceled — remaining in IDLE."
                    )
            else:
                # _on_raw_pose already handled this (cleared tracking_lost,
                # started hop). Clear any residual state defensively.
                self._tracking_lost = False
                self._recovery_map_id_at_loss = None

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
        self._last_auto_cmd_time = time.monotonic()
        future = self._search_client.call_async(req)
        future.add_done_callback(self._on_search_plan_received)

    def _on_search_plan_received(self, future):
        self._search_request_in_flight = False
        if self._state == MissionState.RECALIBRATING:
            self.get_logger().info("Ignoring Search plan response received during recalibration hop.")
            return
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
            self._send_rc_control(0, 0, 0, 0)

    def _on_tof(self, msg):
        """Store the latest ToF reading for use in _handle_recalibration."""
        self._latest_tof_cm = msg.data

    def _start_reinit_calibration_hop(self, new_map_id):
        """
        Enter RECALIBRATING state to manufacture vertical motion so that
        ToFScaleEstimator can accumulate enough delta-based samples for
        the new map_id quickly, rather than waiting passively for a
        Depth-Anything result (which may take up to MAX_ALIGNMENT_GAP_SEC).
        """
        self._recal_map_id = new_map_id
        self._recal_phase = "escape_near_field"
        self._recal_start_time = time.monotonic()
        self._recal_baseline_tof_cm = None
        self._recal_wait_start = None
        self._nbv_request_in_flight = False
        self._search_request_in_flight = False
        self._state = MissionState.RECALIBRATING
        self.get_logger().info(
            f"map_id {new_map_id}: starting calibration hop "
            f"(phase=escape_near_field, timeout=8s)."
        )

    def _handle_recalibration(self):
        """
        Runs from _control_loop while self._state == MissionState.RECALIBRATING.
        Drives the three-phase calibration hop:
          escape_near_field -> climb_delta -> wait_scale -> _finish_recalibration
        A hard HOP_TIMEOUT_SEC cap terminates the whole maneuver if something
        goes wrong (bad ToF, out-of-range readings, etc.).
        """
        HOP_TIMEOUT_SEC = 8.0           # hard safety cap on the whole maneuver
        CLIMB_SPEED = 20                # gentle climb (not MAX_AUTO_SPEED_Z)
        TARGET_MARGIN_CM = 45           # above TOF_MIN_VALID_CM (35), safety buffer
        DELTA_TARGET_CM = 30            # comfortably above MIN_DELTA_TOF_M (20 cm)
        WAIT_SCALE_TIMEOUT_SEC = 5.0

        elapsed = time.monotonic() - self._recal_start_time
        if elapsed > HOP_TIMEOUT_SEC:
            self.get_logger().warn(
                f"map_id {self._recal_map_id}: calibration hop timed out after "
                f"{elapsed:.1f}s - resuming without forced scale."
            )
            self._finish_recalibration()
            return

        tof_cm = self._latest_tof_cm

        # Direction guard: prefer ascending (near-field clamp means we're close
        # to the floor). If somehow we're very high, descend instead.
        # planner_params.yaml bbx.maxZ: 5.0 m -> 500 cm; leave 100 cm headroom.
        CEILING_LIMIT_CM = 400
        climb_dir = -1 if (tof_cm is not None and tof_cm > CEILING_LIMIT_CM) else 1
        climb_cmd = CLIMB_SPEED * climb_dir

        if self._recal_phase == "escape_near_field":
            if tof_cm is not None and tof_cm >= TARGET_MARGIN_CM:
                self._recal_baseline_tof_cm = tof_cm
                self._recal_phase = "climb_delta"
                self.get_logger().info(
                    f"map_id {self._recal_map_id}: ToF {tof_cm:.0f} cm >= "
                    f"{TARGET_MARGIN_CM} cm — switching to climb_delta phase."
                )
            else:
                self._send_rc_control(0, 0, climb_cmd, 0)
            return

        if self._recal_phase == "climb_delta":
            if tof_cm is not None and self._recal_baseline_tof_cm is not None:
                delta = abs(tof_cm - self._recal_baseline_tof_cm)
                if delta >= DELTA_TARGET_CM:
                    self._send_rc_control(0, 0, 0, 0)  # hover, let samples accumulate
                    self._recal_phase = "wait_scale"
                    self._recal_wait_start = time.monotonic()
                    self.get_logger().info(
                        f"map_id {self._recal_map_id}: delta ToF {delta:.0f} cm >= "
                        f"{DELTA_TARGET_CM} cm — switching to wait_scale phase."
                    )
                    return
            self._send_rc_control(0, 0, climb_cmd, 0)
            return

        if self._recal_phase == "wait_scale":
            self._send_rc_control(0, 0, 0, 0)
            scale_ready = (
                self._scale_factor_manager is not None
                and self._scale_factor_manager.current_scale_factors.get(self._recal_map_id) is not None
            )
            if scale_ready:
                self.get_logger().info(
                    f"map_id {self._recal_map_id}: scale ready via forced hop — resuming exploration."
                )
                self._finish_recalibration()
            elif time.monotonic() - self._recal_wait_start > WAIT_SCALE_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"map_id {self._recal_map_id}: scale not ready after hop wait — resuming anyway."
                )
                self._finish_recalibration()

    def _finish_recalibration(self):
        """
        Exits RECALIBRATING state: clear all hop vars, transition to IDLE,
        and kick off exploration on the new map.
        """
        self._recal_map_id = None
        self._recal_phase = None
        self._recal_start_time = None
        self._recal_baseline_tof_cm = None
        self._recal_wait_start = None
        self._state = MissionState.IDLE
        self.start_exploration()

    def _control_loop(self):
        """
        Runs at GOALS_LOOP_RATE_HZ. Checks state and sends velocities.
        """
        if self._state == MissionState.IDLE:
            return

        # ---- Recalibration hop: runs independently of tracking-loss logic ----
        if self._state == MissionState.RECALIBRATING:
            self._handle_recalibration()
            return

        # ---- Tracking-loss detection ----
        # If we have never received a pose yet, just hover.
        if not self._has_pose:
            self._send_rc_control(0, 0, 0, 0)
            return

        # Check staleness of the last pose.
        now = time.monotonic()
        pose_age = now - self._last_pose_time

        if pose_age > self._TRACKING_TIMEOUT_SEC:
            # Tracking is lost. Enter (or stay in) recovery step/hold/check loop.
            if not self._tracking_lost:
                self.get_logger().warn(
                    "SLAM tracking lost! Starting step-check-repeat sweep "
                    f"({self._RECOVERY_YAW_STEP_DEG}° steps, "
                    f"{self._RECOVERY_STEP_HOLD_SEC:.1f}s hold each)."
                )
                self._tracking_lost = True
                self._recovery_map_id_at_loss = self._current_map_id
                self._recovery_turning = True
                self._recovery_yaw_elapsed = 0.0
                self._recovery_step_start = now

            dt = 1.0 / constants.GOALS_LOOP_RATE_HZ

            # Tello at yaw_speed=20 ≈ ~36 deg/s. Scale turn duration from
            # the empirical 90°/2.5s rate: deg / 36 deg_per_sec.
            _TURN_DURATION_SEC = self._RECOVERY_YAW_STEP_DEG / 36.0

            # --- Turning phase: rotate the next small step ---
            if self._recovery_turning:
                self._recovery_yaw_elapsed += dt
                if self._recovery_yaw_elapsed >= _TURN_DURATION_SEC:
                    # Done rotating — begin the hold/check dwell.
                    self._recovery_turning = False
                    self._recovery_step_start = now
                    self.get_logger().info(
                        f"Recovery: holding {self._RECOVERY_STEP_HOLD_SEC:.1f}s "
                        "to check for relocalization..."
                    )
                self._send_rc_control(0, 0, 0, self._RECOVERY_YAW_SPEED)
                return

            # --- Hold/check phase: hover while SLAM tries to re-acquire ---
            # If _pose_callback fires during this window it clears _tracking_lost,
            # and the pose_age check at the top of the next tick will be False,
            # exiting this branch without any extra detection logic here.
            hold_elapsed = now - self._recovery_step_start
            if hold_elapsed < self._RECOVERY_STEP_HOLD_SEC:
                self._send_rc_control(0, 0, 0, 0)
                return

            # Dwell finished and still lost — rotate another small step.
            self._recovery_turning = True
            self._recovery_yaw_elapsed = 0.0
            self.get_logger().info(
                f"Still lost after {self._RECOVERY_STEP_HOLD_SEC:.1f}s — "
                f"rotating another {self._RECOVERY_YAW_STEP_DEG}°..."
            )
            self._send_rc_control(0, 0, 0, self._RECOVERY_YAW_SPEED)
            return

        # ---- Normal flight ----
        if self._tracker.is_done():
            if self._state == MissionState.EXPLORING:
                self.get_logger().info("Reached end of exploration path. Requesting next view...")
                # Temporarily hover while waiting for next path
                self._send_rc_control(0, 0, 0, 0)
                self.start_exploration()
            else:
                self.get_logger().info("Mission complete! Hovering.")
                self._state = MissionState.IDLE
                self._send_rc_control(0, 0, 0, 0)
            return

        cmd_lr, cmd_fb, cmd_ud, cmd_yaw = self._tracker.compute_velocity_commands(
            self._current_x, self._current_y, self._current_z, self._current_yaw
        )

        # Send to drone.
        # send_rc_control silently fails (returns False) if a takeoff/land is in progress,
        # which is exactly the safety behavior we want.
        self._send_rc_control(cmd_lr, cmd_fb, cmd_ud, cmd_yaw)

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
