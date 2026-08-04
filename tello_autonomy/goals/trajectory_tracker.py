"""
goals/trajectory_tracker.py

Pure Python (no ROS2 dependencies) logic for tracking a sequence of waypoints.
Uses a Proportional (P) controller to compute velocity commands based on the
error between the drone's current pose and the target waypoint.

COORDINATE FRAME (ORB-SLAM3 convention):
  World frame as published by ORB-SLAM3:
    Z = Forward  (the direction the camera lens points at initialisation)
    X = Right
    Y = Down (gravity direction)

  Tello send_rc_control(left_right, forward_back, up_down, yaw):
    forward_back: + = forward, - = back
    left_right:   + = right,   - = left
    up_down:      + = up,      - = down
    yaw:          + = CW,      - = CCW

  Therefore the mapping is:
    body_forward =  world_Z component projected into body frame
    body_right   =  world_X component projected into body frame
    body_up      = -world_Y  (Y is DOWN in SLAM, UP for the drone is -Y)

BRAKING:
  To prevent overshoot (the Tello has almost zero drag), we scale the
  commanded speed down linearly when within BRAKING_RADIUS_M of the
  target. At WAYPOINT_ACCEPTANCE_RADIUS_M the drone has already slowed
  nearly to zero before we pop the waypoint.
"""

import math
from config import constants


class TrajectoryTracker:
    def __init__(self):
        self.path = []  # List of dicts: {'x': float, 'y': float, 'z': float}

    def set_path(self, path_poses):
        """
        Receives a list of dicts with 'x', 'y', 'z' keys (from PoseStamped messages).
        """
        self.path = list(path_poses)

    def clear_path(self):
        self.path = []

    def is_done(self):
        return len(self.path) == 0

    def compute_velocity_commands(self, current_x, current_y, current_z, current_yaw_rad):
        """
        Returns (left_right, forward_back, up_down, yaw_velocity) to send to the drone.
        Returns (0, 0, 0, 0) if no path is set or if the path is complete.

        current_x, current_y, current_z: ORB-SLAM3 world coordinates
            (X=Right, Y=Down, Z=Forward)
        current_yaw_rad: drone yaw extracted from the SLAM quaternion (rotation
            around world Y axis, i.e. the vertical/gravity axis)
        """
        if self.is_done():
            return 0, 0, 0, 0

        # Pop AT MOST ONE waypoint per call, regardless of how many technically
        # fall within WAYPOINT_ACCEPTANCE_RADIUS_M. Bulk-popping caused the path
        # to appear "complete" almost instantly relative to the drone's actual
        # position, triggering constant replanning before the drone had time to
        # execute a turn toward any single target.
        if self.path:
            target = self.path[0]
            dz = target['z'] - current_z
            dx = target['x'] - current_x
            dy = target['y'] - current_y
            distance_to_target = math.sqrt(dz**2 + dx**2 + dy**2)

            if distance_to_target < constants.WAYPOINT_ACCEPTANCE_RADIUS_M:
                if len(self.path) == 1 and 'yaw' in target:
                    yaw_err = target['yaw'] - current_yaw_rad
                    yaw_err = (yaw_err + math.pi) % (2.0 * math.pi) - math.pi
                    if abs(yaw_err) <= 0.17:
                        self.path.pop(0)
                else:
                    self.path.pop(0)

        if self.is_done():
            return 0, 0, 0, 0

        # FIX Bug4: recompute dx/dy/dz and distance_to_target from the *current*
        # target after the pop-loop. The values computed inside the while loop above
        # may belong to the waypoint that was just popped, not the new head of the queue.
        target = self.path[0]
        dz = target['z'] - current_z
        dx = target['x'] - current_x
        dy = target['y'] - current_y
        distance_to_target = math.sqrt(dz**2 + dx**2 + dy**2)

        # ---------------------------------------------------------------
        # Braking: linearly scale speed to zero as we approach the target.
        # Outside BRAKING_RADIUS_M → full speed.
        # Inside  BRAKING_RADIUS_M → speed proportional to remaining distance.
        # ---------------------------------------------------------------
        braking_radius = constants.BRAKING_RADIUS_M
        if distance_to_target < braking_radius:
            brake_scale = distance_to_target / braking_radius
        else:
            brake_scale = 1.0

        effective_xy_gain = constants.XY_P_GAIN * brake_scale
        effective_z_gain  = constants.Z_P_GAIN  * brake_scale

        # ---------------------------------------------------------------
        # World-frame velocities (still in SLAM world frame)
        # ---------------------------------------------------------------
        v_world_z = dz * effective_xy_gain   # forward is +Z
        v_world_x = dx * effective_xy_gain   # right   is +X
        v_world_y = dy * effective_z_gain    # down    is +Y  → up = -Y

        # ---------------------------------------------------------------
        # Turn-Then-Move Logic:
        # If the target is far enough away, ensure we face the direction
        # of travel first. If the heading error is large, ONLY yaw.
        # ---------------------------------------------------------------
        travel_yaw = math.atan2(dx, dz)
        
        # Determine the target yaw for this frame
        if distance_to_target > constants.WAYPOINT_ACCEPTANCE_RADIUS_M:
            # We are translating. Target the direction of travel.
            target_yaw = travel_yaw
        elif 'yaw' in target:
            # We arrived at the last waypoint, turn to the planner's desired yaw.
            target_yaw = target['yaw']
        else:
            # Fallback
            target_yaw = travel_yaw

        yaw_err = target_yaw - current_yaw_rad
        yaw_err = (yaw_err + math.pi) % (2.0 * math.pi) - math.pi
        cmd_yaw = int(self._clamp(yaw_err * constants.YAW_P_GAIN, constants.MAX_AUTO_SPEED_YAW))

        # If we need to turn significantly (> 25 degrees) to face the direction
        # of travel, do NOT translate. Just turn.
        if distance_to_target > constants.WAYPOINT_ACCEPTANCE_RADIUS_M and abs(yaw_err) > 0.43: # ~25 deg
            v_forward = 0
            v_right = 0
            v_up = 0
        else:
            # ---------------------------------------------------------------
            # World → Body frame rotation around the gravity axis (world Y).
            # ORB-SLAM3 yaw is a rotation about Y.
            # After rotation:
            #   body_forward  (+Z_body) =  world_Z*cos(yaw) + world_X*sin(yaw)
            #   body_right    (+X_body) = -world_Z*sin(yaw) + world_X*cos(yaw)
            #   body_up       (+Y_body) = -world_Y  (Y_world points down)
            # ---------------------------------------------------------------
            cos_yaw = math.cos(current_yaw_rad)
            sin_yaw = math.sin(current_yaw_rad)

            v_forward = v_world_z * cos_yaw + v_world_x * sin_yaw
            v_right   = -v_world_z * sin_yaw + v_world_x * cos_yaw
            v_up      = -v_world_y            # world Y is down; drone up_down is positive=up

        # ---------------------------------------------------------------
        # Clamp to safe autonomous speeds
        # ---------------------------------------------------------------
        cmd_fb  = int(self._clamp(v_forward, constants.MAX_AUTO_SPEED_XY))
        cmd_lr  = int(self._clamp(v_right,   constants.MAX_AUTO_SPEED_XY))
        cmd_ud  = int(self._clamp(v_up,      constants.MAX_AUTO_SPEED_Z))

        return cmd_lr, cmd_fb, cmd_ud, cmd_yaw

    def _clamp(self, value, max_abs):
        return max(min(value, max_abs), -max_abs)
