"""
drone_interface/tello_driver.py

Owns ONLY the connection lifecycle to the physical Tello drone:
connecting, video stream setup (resolution/fps/bitrate), and shutdown.
Nothing about sending flight commands, reading frames, or reading
telemetry lives here - those are frame_receiver.py, command_handler.py,
and telemetry.py, all of which depend on this module rather than the
other way around.

This module exposes two things every other drone_interface module needs:
  - self.tello       : the underlying djitellopy Tello instance
  - self.cmd_lock     : a shared lock guarding djitellopy's command
                         channel (djitellopy does NOT serialize this
                         itself - concurrent command calls from
                         different threads can corrupt in-flight state)

Modules that only READ state that arrives via the drone's continuous
state broadcast (e.g. telemetry.py's battery/height reads, and
frame_receiver.py's frame reads) do NOT need cmd_lock - only code that
actively SENDS a command over the command channel does.
"""

import threading

from djitellopy import Tello

from config import constants


class TelloDriver:
    """
        Owns the Tello connection itself. Call connect() once at
        startup, hand this instance to frame_receiver / telemetry /
        command_handler, and call disconnect() once at shutdown.
    """

    def __init__(self, resolution="", fps="", bitrate=constants.TELLO_VIDEO_BITRATE):
        """
            resolution: "high" (720p) / "low" (480p) / "" for drone default
            fps: "high"/"middle"/"low" (30/15/5fps) / "" for drone default
            bitrate: name of a Tello.BITRATE_* constant (e.g. "BITRATE_5MBPS"),
                     matching config.constants.TELLO_VIDEO_BITRATE
        """
        self.resolution = resolution
        self.fps = fps
        self.bitrate = bitrate

        # Shared across drone_interface modules - guards every direct
        # call into djitellopy's command channel.
        self.cmd_lock = threading.Lock()

        self.tello = Tello()
        self.frame_reader = None  # set once connected, in connect()

        self._connected = False

    # ****************************************************************************************
    def connect(self):
        """
            Connects to the drone, applies video resolution/fps/bitrate
            settings, and starts the video stream. Call this once
            before handing this instance to any other drone_interface
            module.
        """
        with self.cmd_lock:
            self.tello.connect()
        print(f"Battery: {self.tello.get_battery()}%")

        if self.resolution:
            try:
                with self.cmd_lock:
                    self.tello.set_video_resolution(self.resolution)
            except Exception as e:
                print(f"Could not set video resolution: {e}")

        if self.fps:
            try:
                with self.cmd_lock:
                    self.tello.set_video_fps(self.fps)
            except Exception as e:
                print(f"Could not set video fps: {e}")

        if self.bitrate:
            try:
                bitrate_value = getattr(Tello, self.bitrate)
                with self.cmd_lock:
                    self.tello.set_video_bitrate(bitrate_value)
            except AttributeError:
                print(f"Unknown bitrate constant '{self.bitrate}' on djitellopy.Tello - skipping")
            except Exception as e:
                print(f"Could not set video bitrate: {e}")

        with self.cmd_lock:
            self.tello.streamon()

        # Background thread (owned by djitellopy) keeps exactly ONE
        # decoded frame around, overwriting rather than queueing - so
        # reading it from multiple threads (frame_receiver.py) is safe
        # without needing cmd_lock.
        self.frame_reader = self.tello.get_frame_read()

        self._connected = True

    # ****************************************************************************************
    def is_connected(self):
        return self._connected

    # ****************************************************************************************
    def disconnect(self):
        """
            Stops the video stream and ends the connection. Safe to call
            multiple times. Does NOT land the drone - that's
            command_handler.py's responsibility, and must happen before
            this is called.
        """
        try:
            with self.cmd_lock:
                self.tello.streamoff()
        except Exception:
            pass
        try:
            with self.cmd_lock:
                self.tello.end()
        except Exception:
            pass
        self._connected = False
    # ****************************************************************************************