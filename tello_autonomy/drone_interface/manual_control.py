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

import cv2
from pynput import keyboard


# Movement keys: w/s pitch, a/d roll, i/k throttle, j/l yaw, t takeoff, q land+quit.
MOVE_KEYS = list("wsadikjl")
CONTROL_LOOP_HZ = 30  # how often the loop reads held keys + sends RC control / redraws preview


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

        # Called (if set) once landing completes and this loop is about
        # to stop - lets the owning program (e.g. a ROS2 node) shut
        # itself down too, without this module needing to know anything
        # about ROS2.
        self.on_quit_requested = None

    # ****************************************************************************************
    def start(self, window_name="Tello Manual Control"):
        """
            Starts the pynput keyboard listener and the control loop,
            each on their own thread. Safe to call once.
        """
        self._shutdown.clear()

        self._kb_listener = keyboard.Listener(
            on_press=self._on_key_press,
            on_release=self._on_key_release,
        )
        self._kb_listener.start()

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
            if key.char in MOVE_KEYS or key.char in ("q", "t"):
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
        print("\n" + "=" * 40)
        print("MANUAL FLIGHT CONTROLS (keep this window focused):")
        print("  t: Takeoff   q: Land + Quit")
        print("  w/s: Pitch Fwd/Back   a/d: Roll Left/Right")
        print("  i/k: Throttle Up/Down   j/l: Yaw Left/Right")
        print("=" * 40 + "\n")

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
                cv2.imshow(window_name, cv2.resize(frame, (640, 480)))
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
                if "w" in self._held_keys: fb = self.manual_speed
                if "s" in self._held_keys: fb = -self.manual_speed
                if "a" in self._held_keys: lr = self.manual_speed
                if "d" in self._held_keys: lr = -self.manual_speed
                if "i" in self._held_keys: ud = self.manual_speed
                if "k" in self._held_keys: ud = -self.manual_speed
                if "j" in self._held_keys: yv = -self.manual_speed
                if "l" in self._held_keys: yv = self.manual_speed

                self._command_handler.send_rc_control(lr, fb, ud, yv)

            self._shutdown.wait(timeout=loop_period)

        cv2.destroyAllWindows()
    # ****************************************************************************************