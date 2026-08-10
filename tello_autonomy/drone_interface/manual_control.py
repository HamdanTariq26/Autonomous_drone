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

Requires: pip install pynput
"""

import threading
import time

import cv2
from pynput import keyboard


# Movement keys: w/s pitch, a/d roll, i/k throttle, j/l yaw, t takeoff, q land+quit.
MOVE_KEYS = list("wsadikjl")
CONTROL_LOOP_HZ = 20  # how often the loop reads held keys + sends RC control / redraws preview
# Send RC command every tick (including 0,0,0,0 hover) so the Tello's
# 15-second auto-land watchdog is continuously reset. Never skip the hover
# command - the watchdog fires on ABSENCE of commands, not repeated ones.


class ManualControl:
    def __init__(self, command_handler, frame_receiver, manual_speed=50):
        """
            command_handler: a drone_interface.command_handler.CommandHandler instance
            frame_receiver: a drone_interface.frame_receiver.FrameReceiver instance
            manual_speed: cm/s used for all manual RC movement
        """
        self._command_handler = command_handler
        self._frame_receiver = frame_receiver
        self.manual_speed = manual_speed

        self._held_keys = set()
        self._kb_listener = None
        self._loop_thread = None
        self._shutdown = threading.Event()

        # Called to check if an autonomous mission currently owns the RC
        # channel. When this returns True and no manual key is held, this
        # loop must NOT call send_rc_control at all - mission_controller's
        # own 10Hz loop is already sending real velocity commands, and a
        # second unconditional zero-command from here would race it and
        # intermittently overwrite real commands with hover-zero.
        self.is_autonomous_active = None

        # Optional: callable returning seconds since the autonomous layer last sent
        # an RC command. If autonomous is active but hasn't sent a command for >10s
        # (e.g. planner is computing), manual_control will send a (0,0,0,0) keepalive.
        self.get_auto_time_since_last_cmd = None

        # Called (if set) once landing completes and this loop is about
        # to stop - lets the owning program (e.g. a ROS2 node) shut
        # itself down too, without this module needing to know anything
        # about ROS2.
        self.on_quit_requested = None

        # Called when any flight-control key is pressed. Used to notify
        # autonomous layers to yield control to the human pilot.
        self.on_manual_override = None
        
        # Called when 'h' is pressed to trigger Return-to-Home
        self.on_rth_requested = None

        # Called when 'e' is pressed to trigger/toggle autonomous NBV exploration
        self.on_explore_requested = None

        # Called when 'm' is pressed to cycle the active scale-factor
        # source (ToF / Depth-Anything / Auto).
        self.on_scale_mode_cycle_requested = None

        # Persistent status text for scale factor
        self._scale_status_text = "SLAM: Waiting..."
        self._scale_status_color = (255, 255, 255)  # white

    def set_scale_status(self, text, color):
        self._scale_status_text = text
        self._scale_status_color = color

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
            ch = key.char.lower() if getattr(key, "char", None) else None
            if ch == 'h':
                if self.on_rth_requested is not None:
                    self.on_rth_requested()
            elif ch == 'e':
                if self.on_explore_requested is not None:
                    self.on_explore_requested()
            elif ch == 'm':
                if self.on_scale_mode_cycle_requested is not None:
                    self.on_scale_mode_cycle_requested()
            elif (ch in MOVE_KEYS or ch in ("q", "t")) and ch is not None:
                self._held_keys.add(ch)
                if self.on_manual_override is not None:
                    self.on_manual_override()
        except Exception:
            pass

        # Handle special keys (Arrow keys, Spacebar, ESC)
        if hasattr(key, 'name'):
            name = key.name
            if name in ("up", "down", "left", "right", "space", "esc"):
                self._held_keys.add(name)
                if self.on_manual_override is not None:
                    self.on_manual_override()

    def _on_key_release(self, key):
        try:
            ch = key.char.lower() if getattr(key, "char", None) else None
            if ch:
                self._held_keys.discard(ch)
        except Exception:
            pass

        if hasattr(key, 'name'):
            self._held_keys.discard(key.name)
    # ****************************************************************************************

    # ****************************************************************************************
    def _control_loop(self, window_name):
        """
            Runs on its own thread. Owns the preview window. Reads key
            state from self._held_keys (kept current by the pynput
            listener's own thread) every iteration and sends RC control
            accordingly - never uses cv2.waitKey() for key state, only
            to let the preview window paint/refresh and process its own
            window events.
        """
        cv2.namedWindow(window_name)
        print("\n" + "=" * 50)
        print("MANUAL FLIGHT CONTROLS (keep this window focused):")
        print("  t: Takeoff   q: Land + Quit   h: Return Home   e: Explore (NBV)   m: Cycle Scale Source")
        print("  w/s: Pitch Fwd/Back   a/d: Roll Left/Right")
        print("  i/k: Throttle Up/Down   j/l: Yaw Left/Right")
        print("=" * 50 + "\n")

        took_off = False  # edge-detect guard so holding 't' doesn't spam takeoff
        loop_period = 1.0 / CONTROL_LOOP_HZ

        def _on_land_complete(success, error):
            # Runs on command_handler's worker thread, not this loop's
            # thread - only touches thread-safe primitives.
            self._shutdown.set()
            if self.on_quit_requested is not None:
                self.on_quit_requested()

        while not self._shutdown.is_set():
            frame = self._frame_receiver.get_frame_bgr()
            if frame is not None:
                display_frame = cv2.resize(frame, (640, 480))
                if self._scale_status_text:
                    cv2.putText(display_frame, self._scale_status_text, (20, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, self._scale_status_color, 2, cv2.LINE_AA)
                cv2.imshow(window_name, display_frame)
            cv2.waitKey(1)  # only paints/refreshes the window - not used for key state

            if "q" in self._held_keys and not self._command_handler.is_landing_in_progress():
                self._command_handler.land_async(on_complete=_on_land_complete)

            if ("t" in self._held_keys and not self._command_handler.is_flying and not took_off
                    and not self._command_handler.is_takeoff_in_progress()
                    and not self._command_handler.is_landing_in_progress()):
                took_off = True
                self._command_handler.takeoff_async()
            elif "t" not in self._held_keys:
                took_off = False

            if (self._command_handler.is_flying
                    and not self._command_handler.is_landing_in_progress()
                    and not self._command_handler.is_takeoff_in_progress()):
                lr = fb = ud = yv = 0
                if "w" in self._held_keys or "up" in self._held_keys: fb = self.manual_speed
                if "s" in self._held_keys or "down" in self._held_keys: fb = -self.manual_speed
                if "a" in self._held_keys or "left" in self._held_keys: lr = -self.manual_speed
                if "d" in self._held_keys or "right" in self._held_keys: lr = self.manual_speed
                if "i" in self._held_keys: ud = self.manual_speed
                if "k" in self._held_keys: ud = -self.manual_speed
                if "j" in self._held_keys: yv = -self.manual_speed
                if "l" in self._held_keys: yv = self.manual_speed

                any_key_held = (lr != 0 or fb != 0 or ud != 0 or yv != 0)
                autonomous_active = (
                    self.is_autonomous_active is not None
                    and self.is_autonomous_active()
                )

                auto_silent_timeout = False
                if autonomous_active and self.get_auto_time_since_last_cmd is not None:
                    if self.get_auto_time_since_last_cmd() > 10.0:
                        auto_silent_timeout = True

                if any_key_held or not autonomous_active or auto_silent_timeout:
                    # Either the human is actively flying, or nothing
                    # autonomous is running, or autonomous has been silent
                    # for >10 seconds (resettable watchdog) - this loop
                    # is the sole authority over the RC channel and must keep
                    # sending every tick (including hover-zero) to reset
                    # the Tello's 15s auto-land watchdog.
                    self._command_handler.send_rc_control(lr, fb, ud, yv)
                # else: an autonomous mission owns the channel and no key
                # is held - stay silent so mission_controller's own 10Hz
                # loop is the only thing writing to send_rc_control.

            self._shutdown.wait(timeout=loop_period)

        cv2.destroyAllWindows()
    # ****************************************************************************************