"""
drone_interface/command_handler.py

The ONLY module in drone_interface allowed to actually send flight
commands. manual_control.py, and any future autonomous flight logic,
must go through this - never call tello_driver.tello.<command>()
directly from anywhere else.

Depends only on drone_interface.tello_driver (for the shared Tello
instance and cmd_lock). No frame reading, no keyboard/UI code - that
belongs to frame_receiver.py and manual_control.py respectively.

Design notes carried over from earlier prototyping, still true here:
  - takeoff()/land() can block up to ~21s worst case (djitellopy:
    RESPONSE_TIMEOUT=7s x RETRY_COUNT=3). They're dispatched onto their
    own short-lived worker threads (takeoff_async/land_async) so
    calling code (a UI loop, a ROS2 callback) never freezes waiting on
    their ACK.
  - send_rc_control() is fire-and-forget (no ACK wait) by design -
    intended for continuous manual/velocity control, not queried for
    success.
  - run_command() is a generic, lock-guarded dispatcher for any other
    djitellopy method (move_forward, rotate_clockwise, go_xyz_speed,
    etc.) by name - avoids hand-writing a wrapper for every command
    before we know which ones autonomous flight logic will actually
    need. It's a BLOCKING call (same as the underlying djitellopy
    method) - callers running it from a UI/ROS2 loop are responsible
    for dispatching it onto their own thread if that matters to them.
"""

import threading

KEEPALIVE_INTERVAL_SEC = 10.0


class CommandHandler:
    def __init__(self, tello_driver):
        """
            tello_driver: a drone_interface.tello_driver.TelloDriver
                          instance, already connected.
        """
        self._tello_driver = tello_driver

        self._takeoff_in_progress = threading.Event()
        self._landing_in_progress = threading.Event()

        self._keepalive_thread = None
        self._keepalive_shutdown = threading.Event()

    # ****************************************************************************************
    @property
    def is_flying(self):
        return self._tello_driver.tello.is_flying

    def is_takeoff_in_progress(self):
        return self._takeoff_in_progress.is_set()

    def is_landing_in_progress(self):
        return self._landing_in_progress.is_set()
    # ****************************************************************************************

    # ****************************************************************************************
    def takeoff_async(self, on_complete=None):
        """
            Dispatches takeoff() on its own thread and returns
            immediately. is_takeoff_in_progress() stays True for the
            whole call, so callers can avoid dispatching a second
            takeoff while this one is still waiting on its ACK.

            on_complete: optional callable(success: bool, error: Exception | None),
                         invoked from the worker thread once takeoff() returns/fails.
            Returns False without doing anything if a takeoff or landing
            is already in progress.
        """
        if self._takeoff_in_progress.is_set() or self._landing_in_progress.is_set():
            return False

        self._takeoff_in_progress.set()
        threading.Thread(target=self._run_takeoff, args=(on_complete,), daemon=True).start()
        return True

    def _run_takeoff(self, on_complete):
        error = None
        try:
            with self._tello_driver.cmd_lock:
                self._tello_driver.tello.takeoff()
        except Exception as e:
            error = e
            print(f"Takeoff failed: {e}")
        finally:
            self._takeoff_in_progress.clear()
        if on_complete is not None:
            on_complete(error is None, error)
    # ****************************************************************************************

    # ****************************************************************************************
    def land_async(self, on_complete=None):
        """
            Dispatches land() on its own thread and returns immediately.
            Also zeroes RC control first (fast, fire-and-forget) so the
            drone stops drifting the instant land is requested, rather
            than waiting for land()'s own ACK cycle.

            on_complete: optional callable(success: bool, error: Exception | None).
            Returns False without doing anything if a landing is already
            in progress.
        """
        if self._landing_in_progress.is_set():
            return False

        self._landing_in_progress.set()
        try:
            self.send_rc_control(0, 0, 0, 0)
        except Exception as e:
            print(f"Zeroing RC control before landing failed (non-fatal): {e}")

        threading.Thread(target=self._run_land, args=(on_complete,), daemon=True).start()
        return True

    def _run_land(self, on_complete):
        error = None
        try:
            with self._tello_driver.cmd_lock:
                self._tello_driver.tello.land()
        except Exception as e:
            error = e
            print(f"Landing failed: {e}")
        finally:
            self._landing_in_progress.clear()
        if on_complete is not None:
            on_complete(error is None, error)
    # ****************************************************************************************

    # ****************************************************************************************
    def send_rc_control(self, left_right, forward_back, up_down, yaw):
        """
            Fire-and-forget continuous velocity control - no ACK wait,
            so this never triggers djitellopy's RESPONSE_TIMEOUT/RETRY_COUNT
            retry warnings the way discrete move_*() commands would if
            spammed. Intended to be called repeatedly (e.g. every loop
            iteration of a manual control or autonomous velocity
            controller), not once.

            Silently refuses (returns False) while a takeoff or landing
            is in progress, to avoid conflicting commands mid-sequence.
            Any djitellopy-level failure is caught and printed rather
            than raised - a single dropped RC packet should never crash
            whatever loop is calling this every frame.
        """
        if self._takeoff_in_progress.is_set() or self._landing_in_progress.is_set():
            return False
        try:
            with self._tello_driver.cmd_lock:
                self._tello_driver.tello.send_rc_control(left_right, forward_back, up_down, yaw)
            return True
        except Exception as e:
            print(f"send_rc_control failed (non-fatal): {e}")
            return False
    # ****************************************************************************************

    # ****************************************************************************************
    def run_command(self, command_name, *args, **kwargs):
        """
            Generic, lock-guarded dispatch of any other djitellopy Tello
            method by name - e.g. run_command("move_forward", 100),
            run_command("rotate_clockwise", 90), run_command("go_xyz_speed", 100, 0, 0, 50).

            BLOCKING - waits for the underlying djitellopy call to
            return (which itself waits for the command's ACK). Callers
            running this from a UI or ROS2 callback loop are responsible
            for dispatching it onto their own thread if blocking there
            would be a problem, the same way takeoff_async/land_async
            do internally.

            Raises AttributeError if command_name isn't a real method on
            djitellopy's Tello class - deliberately not swallowed, since
            a typo'd command name during autonomous flight logic should
            fail loudly during development, not silently do nothing.
        """
        method = getattr(self._tello_driver.tello, command_name)
        with self._tello_driver.cmd_lock:
            return method(*args, **kwargs)
    # ****************************************************************************************

    # ****************************************************************************************
    def start_keepalive(self):
        """
            Starts the keepalive loop on its own background thread. Safe
            to call once, after the TelloDriver is connected.
        """
        self._keepalive_shutdown.clear()
        self._keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
        self._keepalive_thread.start()

    def stop_keepalive(self, join_timeout=2.0):
        self._keepalive_shutdown.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=join_timeout)

    def _keepalive_loop(self):
        """
            Runs on its own background thread, never on whatever thread
            is responsible for publishing frames or handling manual
            control. Uses a non-blocking lock acquire so it skips a
            cycle rather than colliding with an in-progress command
            (safe to skip: an in-progress command already reset the
            drone's own auto-land timer).
        """
        while not self._keepalive_shutdown.is_set():
            if self._tello_driver.cmd_lock.acquire(blocking=False):
                try:
                    self._tello_driver.tello.send_keepalive()
                except Exception as e:
                    print(f"keepalive failed (non-fatal): {e}")
                finally:
                    self._tello_driver.cmd_lock.release()
            self._keepalive_shutdown.wait(timeout=KEEPALIVE_INTERVAL_SEC)
    # ****************************************************************************************