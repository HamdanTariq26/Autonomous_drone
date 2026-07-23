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

        # FIX: Both the Tello viewer and SLAM viewer lag because they both read
        # from BackgroundFrameRead._frame, which is produced by PyAV's decode loop.
        #
        # ROOT CAUSE (proven by source code inspection):
        # H264 uses B-frames (bidirectional predicted frames). B-frames cannot be
        # decoded until the NEXT I/P frame arrives, so FFMPEG holds frames in its
        # internal Decoded Picture Buffer (DPB) waiting for the future reference frame.
        # With thread_count=0 (FFMPEG auto-selects, typically 4-8 threads), FFMPEG
        # creates a multi-frame pipeline: it buffers 4-8 frames INSIDE the decoder
        # before yielding ANY to Python. Every time the Python GIL is busy (5 ROS
        # executor threads, cv2.imshow, disk I/O), the decode thread wakes up late,
        # FFMPEG has accumulated more frames in the DPB, and it outputs them IN ORDER
        # from the front of the buffer - it cannot skip to the latest. The lag
        # grows over the entire session because backpressure compounds.
        #
        # THE FIX:
        # 1. fflags=nobuffer + flags=low_delay: stop FFMPEG buffering at the
        #    network/demuxer level (already applied).
        # 2. After av.open(), set skip_frame='BIDIR' on the codec context: tells
        #    FFMPEG to SKIP B-frames entirely - no look-ahead needed, no DPB buildup.
        # 3. Set thread_count=1: eliminates the multi-thread pipeline delay.
        #    With 1 thread, FFMPEG decodes one frame at a time with no internal queue.
        #
        # skip_frame='BIDIR' verified working by:
        #   ctx = av.codec.CodecContext.create('h264', 'r')
        #   ctx.skip_frame = 'BIDIR'  -> prints 'BIDIR' (no exception)
        import av
        original_av_open = av.open

        def custom_av_open(file, format=None, options=None, *args, **kwargs):
            if options is None:
                options = {}
            options.update({
                'fflags': 'nobuffer',
                'flags': 'low_delay',
                'strict': 'experimental',
            })
            container = original_av_open(file, format=format, options=options, *args, **kwargs)

            # After the container is open, patch the video stream's codec context
            # to eliminate DPB pipeline delay. This must happen BEFORE decode() is
            # called (decode() is called inside update_frame() which starts after
            # BackgroundFrameRead.start() is called, which is after __init__ returns).
            try:
                for stream in container.streams.video:
                    stream.codec_context.skip_frame = 'BIDIR'  # skip B-frames
                    stream.codec_context.thread_count = 1       # no multi-thread pipeline
            except Exception as e:
                print(f"[tello_driver] Warning: could not set codec_context options: {e}")

            return container

        av.open = custom_av_open
        self.frame_reader = self.tello.get_frame_read()
        av.open = original_av_open  # restore original

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

        # PROOF-BASED FIX: djitellopy's BackgroundFrameRead.stop() only sets a
        # boolean flag. The background thread checks that flag only BETWEEN frames,
        # but it is permanently blocked on container.decode() which is a blocking
        # generator. If the stream dies or we quit, decode() never yields and the
        # flag is never seen, leaving the thread (and its OS socket on port 11111)
        # alive after the process should have shut down cleanly.
        #
        # Calling container.close() directly forces FFMPEG to abort the decode
        # immediately, which raises an exception in the generator, which breaks
        # the for-loop, which lets the thread exit and release the socket.
        # This is the root cause of the "non-existing PPS 0 referenced / no frame"
        # errors seen on the next restart: the old run's socket was still bound.
        if self.frame_reader is not None:
            try:
                bgfr = self.frame_reader
                bgfr.stopped = True
                if hasattr(bgfr, 'container') and bgfr.container is not None:
                    bgfr.container.close()
                if hasattr(bgfr, 'worker') and bgfr.worker.is_alive():
                    bgfr.worker.join(timeout=2.0)
            except Exception:
                pass

        try:
            with self.cmd_lock:
                self.tello.end()
        except Exception:
            pass
        self._connected = False
    # ****************************************************************************************