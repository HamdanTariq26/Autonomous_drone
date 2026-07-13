"""
drone_interface/manual_control.py

Manual keyboard flight control: preview window + keyboard-to-RC-values
mapping. Depends on command_handler.py (to actually move the drone) and
frame_receiver.py (to show the live preview) - never touches
tello_driver or djitellopy directly. This is the only drone_interface
module that owns a UI window or a keyboard listener.

Input capture uses pynput.keyboard.Listener - real OS-level key
press/release events on their own dedicated thread - rather than
cv2.waitKey() polling. This matters for responsiveness: waitKey()-based
control only samples key state once per preview-window redraw and has
to guess whether a key is "still held" from OS auto-repeat timing,
which is exactly what made manual control feel wonky/delayed before.
pynput gives true key-down/key-up state instead, decoupled entirely
from how fast the preview window is redrawing.

AUTONOMOUS TEST MODE (optional - only when scale_factor_manager and
pose_subscriber are passed in):
  Once the first metric scale factor is computed, an on-screen prompt
  appears. Press 'y' to start a 1-meter forward test flight. Any
  manual flight key (w/a/s/d/i/k/j/l) pressed during autonomous mode
  instantly aborts it and returns full manual control.

Requires: pip install pynput
"""

import math
import threading
import time

import cv2
from pynput import keyboard


# Movement keys: w/s pitch, a/d roll, i/k throttle, j/l yaw, t takeoff, q land+quit.
MOVE_KEYS = list("wsadikjl")
CONTROL_LOOP_HZ = 30  # how often the loop reads held keys + sends RC control / redraws preview

# Autonomous test flight constants
AUTONOMOUS_FORWARD_SPEED = 30   # cm/s - slower than manual for reliable SLAM tracking
AUTONOMOUS_TARGET_DISTANCE_M = 1.0  # meters to travel before stopping

# States for the autonomous test mode state machine
_STATE_MANUAL = "MANUAL"           # normal manual control (default)
_STATE_READY = "READY"             # scale factor computed, awaiting 'y' key press
_STATE_AUTONOMOUS = "AUTONOMOUS"   # executing the 1-meter forward flight


class ManualControl:
    def __init__(self, command_handler, frame_receiver, manual_speed=50,
                 scale_factor_manager=None, pose_subscriber=None):
        """
            command_handler:      a drone_interface.command_handler.CommandHandler instance
            frame_receiver:       a drone_interface.frame_receiver.FrameReceiver instance
            manual_speed:         cm/s used for all manual RC movement
            scale_factor_manager: (optional) perception.scale_factor_manager.ScaleFactorManager -
                                  when provided, enables autonomous test mode once a scale factor
                                  is available. Passed as a plain Python object reference; this
                                  module never imports any ROS2 or perception code.
            pose_subscriber:      (optional) main.PoseSubscriberNode - provides the live metric
                                  pose for distance tracking during autonomous flight.
        """
        self._command_handler = command_handler
        self._frame_receiver = frame_receiver
        self.manual_speed = manual_speed

        # Optional: autonomous test mode wiring (None = feature disabled)
        self._scale_factor_manager = scale_factor_manager
        self._pose_subscriber = pose_subscriber

        self._held_keys = set()
        self._kb_listener = None
        self._loop_thread = None
        self._shutdown = threading.Event()

        # Called (if set) once landing completes and this loop is about
        # to stop - lets the owning program (e.g. a ROS2 node) shut
        # itself down too, without this module needing to know anything
        # about ROS2.
        self.on_quit_requested = None

    # ****************************************************************************************
    def start(self, window_name="Tello Manual Control", blocking=False):
        """
            Starts the pynput keyboard listener and the control loop.
            If blocking=True, the control loop runs in the current thread (required for OpenCV UI on many OSes).
            Otherwise, it runs on its own background thread.
        """
        self._shutdown.clear()

        self._kb_listener = keyboard.Listener(
            on_press=self._on_key_press,
            on_release=self._on_key_release,
        )
        self._kb_listener.start()

        if blocking:
            self._control_loop(window_name)
            return None
        else:
            self._loop_thread = threading.Thread(
                target=self._control_loop, args=(window_name,), daemon=True
            )
            self._loop_thread.start()
            return self._loop_thread

    def stop(self, join_timeout=2.0):
        self._shutdown.set()
        if self._kb_listener is not None:
            self._kb_listener.stop()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=join_timeout)
    # ****************************************************************************************

    # ****************************************************************************************
    def _on_key_press(self, key):
        try:
            if key.char in MOVE_KEYS or key.char in ("q", "t", "y"):
                self._held_keys.add(key.char)
        except AttributeError:
            pass  # special keys (ctrl, shift, alt, etc) - not used, ignore

    def _on_key_release(self, key):
        try:
            self._held_keys.discard(key.char)
        except AttributeError:
            pass
    # ****************************************************************************************

    # ****************************************************************************************
    def _autonomous_feature_enabled(self):
        """Returns True only if both optional dependencies were provided."""
        return self._scale_factor_manager is not None and self._pose_subscriber is not None

    def _has_scale_factor(self):
        """Returns True if at least one map_id has a computed scale factor."""
        if self._scale_factor_manager is None:
            return False
        return bool(self._scale_factor_manager.current_scale_factors)

    def _get_current_xyz(self):
        """
            Returns (x, y, z) in meters from the latest metric pose, or None
            if no pose has been received yet.
        """
        if self._pose_subscriber is None:
            return None
        pose_msg = self._pose_subscriber.get_pose()
        if pose_msg is None:
            return None
        p = pose_msg.pose.position
        return (p.x, p.y, p.z)
    # ****************************************************************************************

    # ****************************************************************************************
    def _draw_hud(self, frame, state, distance_traveled=0.0):
        """
            Draws the in-flight status overlay directly onto `frame` (in-place).
            Shows the current mode and any relevant instructions or progress.
        """
        h, w = frame.shape[:2]

        # --- Mode badge (top-left) ---
        if state == _STATE_MANUAL:
            badge_color = (50, 200, 50)   # green
            badge_text = "MODE: MANUAL"
        elif state == _STATE_READY:
            badge_color = (50, 200, 255)  # yellow/amber
            badge_text = "MODE: MANUAL  [Scale ready!]"
        else:  # AUTONOMOUS
            badge_color = (50, 50, 255)   # red
            badge_text = "MODE: AUTONOMOUS"

        cv2.rectangle(frame, (0, 0), (w, 34), (0, 0, 0), -1)
        cv2.putText(frame, badge_text, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, badge_color, 2, cv2.LINE_AA)

        # --- Autonomous mode: progress bar + instruction ---
        if state == _STATE_AUTONOMOUS:
            progress = min(distance_traveled / AUTONOMOUS_TARGET_DISTANCE_M, 1.0)
            bar_x, bar_y, bar_w, bar_h = 8, 42, w - 16, 18
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (60, 60, 60), -1)
            cv2.rectangle(frame, (bar_x, bar_y),
                          (bar_x + int(bar_w * progress), bar_y + bar_h), (50, 50, 255), -1)
            cv2.putText(frame, f"{distance_traveled:.2f}m / {AUTONOMOUS_TARGET_DISTANCE_M:.1f}m",
                        (bar_x + 4, bar_y + 13),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            abort_text = "Press W/A/S/D/I/K/J/L to ABORT"
            cv2.putText(frame, abort_text, (8, bar_y + bar_h + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 255), 1, cv2.LINE_AA)

        # --- READY state: prompt to start autonomous ---
        elif state == _STATE_READY:
            prompt = "Press  'Y'  to start 1m autonomous test flight"
            cv2.putText(frame, prompt, (8, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 220, 255), 2, cv2.LINE_AA)
    # ****************************************************************************************

    # ****************************************************************************************
    def _control_loop(self, window_name):
        """
            Runs on its own thread (or main thread if blocking=True). Owns the
            preview window. Reads key state from self._held_keys (kept current
            by the pynput listener's own thread) every iteration.

            State machine (only active when scale_factor_manager + pose_subscriber
            are provided):
              _STATE_MANUAL     -> _STATE_READY  when first scale factor is computed
              _STATE_READY      -> _STATE_AUTONOMOUS when 'y' is pressed (while flying)
              _STATE_AUTONOMOUS -> _STATE_MANUAL  when 1m reached, or any move key pressed
        """
        cv2.namedWindow(window_name)
        print("\n" + "=" * 40)
        print("MANUAL FLIGHT CONTROLS (keep this window focused):")
        print("  t: Takeoff   q: Land + Quit")
        print("  w/s: Pitch Fwd/Back   a/d: Roll Left/Right")
        print("  i/k: Throttle Up/Down   j/l: Yaw Left/Right")
        if self._autonomous_feature_enabled():
            print("  [Autonomous test mode enabled - waiting for scale factor...]")
        print("=" * 40 + "\n")

        took_off = False
        loop_period = 1.0 / CONTROL_LOOP_HZ

        # State machine
        state = _STATE_MANUAL
        _scale_notified = False      # have we printed the console notification yet?
        _auto_start_xyz = None       # (x,y,z) when autonomous run started
        _distance_traveled = 0.0

        def _on_land_complete(success, error):
            # Runs on command_handler's worker thread - only touches thread-safe primitives.
            self._shutdown.set()
            if self.on_quit_requested is not None:
                self.on_quit_requested()

        while not self._shutdown.is_set():
            # --- Grab latest frame ---
            frame = self._frame_receiver.get_frame_bgr()

            # --- State machine transitions ---
            if self._autonomous_feature_enabled():
                if state == _STATE_MANUAL and self._has_scale_factor():
                    state = _STATE_READY
                    if not _scale_notified:
                        print("\n" + "=" * 50)
                        print("[AUTONOMOUS TEST MODE]  Scale factor is ready!")
                        print("  Press  'y'  to execute the 1-meter forward test.")
                        print("  Press any movement key at any time to abort.")
                        print("=" * 50 + "\n")
                        _scale_notified = True

                if state == _STATE_READY:
                    # Activate only while drone is actually flying
                    if "y" in self._held_keys and self._command_handler.is_flying:
                        start_xyz = self._get_current_xyz()
                        if start_xyz is not None:
                            state = _STATE_AUTONOMOUS
                            _auto_start_xyz = start_xyz
                            _distance_traveled = 0.0
                            print("[AUTONOMOUS] Starting 1m forward test flight...")
                        else:
                            print("[AUTONOMOUS] No metric pose yet - try again in a moment.")

                if state == _STATE_AUTONOMOUS:
                    # Check for manual override - any move key instantly aborts
                    manual_key_pressed = any(k in self._held_keys for k in MOVE_KEYS)
                    if manual_key_pressed:
                        state = _STATE_READY
                        _auto_start_xyz = None
                        _distance_traveled = 0.0
                        self._command_handler.send_rc_control(0, 0, 0, 0)
                        print("[AUTONOMOUS] Aborted - manual override. Back to manual control.")

            # --- Draw frame + HUD ---
            if frame is not None:
                display = cv2.resize(frame, (640, 480))
                self._draw_hud(display, state, _distance_traveled)
                cv2.imshow(window_name, display)
            cv2.waitKey(1)  # only paints/refreshes the window - not used for key state

            # --- Takeoff / Land logic (always active) ---
            if "q" in self._held_keys and not self._command_handler.is_landing_in_progress():
                if state == _STATE_AUTONOMOUS:
                    # Stop autonomous before landing
                    self._command_handler.send_rc_control(0, 0, 0, 0)
                    state = _STATE_MANUAL
                self._command_handler.land_async(on_complete=_on_land_complete)

            if (not self._command_handler.is_flying
                    and not took_off
                    and "t" in self._held_keys
                    and not self._command_handler.is_takeoff_in_progress()
                    and not self._command_handler.is_landing_in_progress()):
                took_off = True
                self._command_handler.takeoff_async()
            elif "t" not in self._held_keys:
                took_off = False

            # --- RC control: autonomous OR manual ---
            if (self._command_handler.is_flying
                    and not self._command_handler.is_landing_in_progress()
                    and not self._command_handler.is_takeoff_in_progress()):

                if state == _STATE_AUTONOMOUS:
                    # Measure distance traveled from start position
                    current_xyz = self._get_current_xyz()
                    if current_xyz is not None and _auto_start_xyz is not None:
                        dx = current_xyz[0] - _auto_start_xyz[0]
                        dy = current_xyz[1] - _auto_start_xyz[1]
                        dz = current_xyz[2] - _auto_start_xyz[2]
                        _distance_traveled = math.sqrt(dx*dx + dy*dy + dz*dz)

                    if _distance_traveled >= AUTONOMOUS_TARGET_DISTANCE_M:
                        # Target reached - stop and return to manual
                        self._command_handler.send_rc_control(0, 0, 0, 0)
                        self._last_rc = (0, 0, 0, 0)
                        self._last_rc_time = time.time()
                        state = _STATE_READY
                        _auto_start_xyz = None
                        _distance_traveled = 0.0
                        print(f"[AUTONOMOUS] Target reached! Stopped after "
                              f"{AUTONOMOUS_TARGET_DISTANCE_M:.1f}m. Back to manual control.")
                    else:
                        # Keep flying forward
                        current_rc = (0, AUTONOMOUS_FORWARD_SPEED, 0, 0)
                        last_rc = getattr(self, '_last_rc', None)
                        last_rc_time = getattr(self, '_last_rc_time', 0)
                        current_time = time.time()
                        if current_rc != last_rc or current_time - last_rc_time > 0.1:
                            self._command_handler.send_rc_control(0, AUTONOMOUS_FORWARD_SPEED, 0, 0)
                            self._last_rc = current_rc
                            self._last_rc_time = current_time

                else:
                    # Normal manual control
                    lr = fb = ud = yv = 0
                    if "w" in self._held_keys: fb = self.manual_speed
                    if "s" in self._held_keys: fb = -self.manual_speed
                    if "a" in self._held_keys: lr = self.manual_speed
                    if "d" in self._held_keys: lr = -self.manual_speed
                    if "i" in self._held_keys: ud = self.manual_speed
                    if "k" in self._held_keys: ud = -self.manual_speed
                    if "j" in self._held_keys: yv = -self.manual_speed
                    if "l" in self._held_keys: yv = self.manual_speed

                    current_rc = (lr, fb, ud, yv)
                    last_rc = getattr(self, '_last_rc', None)
                    last_rc_time = getattr(self, '_last_rc_time', 0)
                    current_time = time.time()

                    # Only send RC command if the command CHANGED, OR if we are actively
                    # moving and it has been > 0.1s since we last sent it (10Hz).
                    # We do NOT spam 0,0,0,0 while hovering.
                    if current_rc != last_rc or (current_rc != (0, 0, 0, 0) and current_time - last_rc_time > 0.1):
                        self._command_handler.send_rc_control(lr, fb, ud, yv)
                        self._last_rc = current_rc
                        self._last_rc_time = current_time

            self._shutdown.wait(timeout=loop_period)

        cv2.destroyAllWindows()
    # ****************************************************************************************