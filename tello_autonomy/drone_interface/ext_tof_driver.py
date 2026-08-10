"""
drone_interface/ext_tof_driver.py

Driver for the Tello Extension Kit's downward Matrix/ToF sensor,
queried via the SDK 3.0 "EXT tof?" command
(tello.send_command_with_return("EXT tof?")).

This is a DIFFERENT physical sensor from the built-in bottom ToF that
drone_interface.telemetry.TelemetryMonitor already reads via
tello.get_distance_tof() (the state-stream field "tof", cm precision,
~10-800cm range). The extension-kit sensor:
  - Replies in MILLIMETERS on the synchronous command channel, not the
    async state stream.
  - Is rated +/-4mm accurate, usable range up to ~1.2m (1200mm).
  - Returns a sentinel in the 8190-8192 range when out of range/no
    target detected (mirrors the same sentinel pattern the built-in
    ToF uses in perception/tof_scale_estimator.py's is_tof_valid()).
  - Can occasionally hang for several seconds on
    send_command_with_return() (observed up to ~7s) - this is a
    blocking synchronous command, unlike the state-stream field, so it
    must NEVER be polled from a latency-sensitive thread (flight
    control, a ROS2 executor callback). It gets its own dedicated
    thread here, the same way command_handler.py already dispatches
    slow blocking Tello SDK calls (takeoff/land) onto their own
    threads rather than the caller's.

Threading model: this is a plain background thread (not a
multiprocessing.Process). The workload is I/O-bound (blocked on a UDP
round-trip via djitellopy's response-wait/retry loop), not CPU-bound,
so it does not starve other threads' CPU time the way a CPU-bound
model-inference workload would (contrast with
perception/depth_inference_worker.py, which DOES need a separate OS
process + os.nice() specifically because DepthAnything inference is
CPU-bound and would otherwise compete with flight control for the same
process's CPU time). A thread is sufficient and simpler here.

Concurrency note: send_command_with_return() goes over the same
command channel as everything in command_handler.py. This driver
therefore takes tello_driver.cmd_lock for the duration of each query,
same as every other command-channel caller - it must never run
un-synchronized against takeoff/land/run_command.

This module has ONLY driver responsibilities: connect, poll, parse,
detect staleness, expose the latest reading. It does not do any scale
estimation math - see perception/ext_tof_scale_estimator.py for that
(same separation telemetry.py/tof_scale_estimator.py already use for
the built-in sensor).
"""

import re
import threading
import time


# Sentinel band for "out of range / no target". The exact boundary
# varies slightly by firmware in the wild, so treat the whole
# 8190-8192 neighborhood as invalid rather than a single exact value.
EXT_TOF_SENTINEL_MIN_MM = 8190

# Sensor is rated to ~1.2m (1200mm). Leave margin below the rated max
# so readings right at the edge of the rated range are excluded, this
# mirrors constants.TOF_MAX_VALID_CM's pattern of sitting comfortably
# inside the sensor's real usable band rather than at its exact limit.
EXT_TOF_MAX_VALID_MM = 1150.0

# Below this, treat as near-field noise / not physically meaningful
# for a drone hovering indoors. No datasheet-specified near-field
# clamp is documented for this sensor the way the built-in ToF has
# one, so this is a conservative floor rather than a measured clamp.
EXT_TOF_MIN_VALID_MM = 20.0

# If no successful reading (valid or invalid-but-parsed) has landed in
# this long, the sensor is considered STALE - covers both the
# documented "hangs up to ~7s" behavior and a genuinely dead
# connection. Set comfortably above the observed worst-case hang so a
# single slow query doesn't flap liveness, but well under
# "indistinguishable from disconnected."
EXT_TOF_STALE_AFTER_SEC = 10.0

# Polling cadence when queries are completing normally. Each query
# itself may take anywhere from ~50ms to several seconds (hang case),
# so this is a "wait at most this long between the END of one query
# and the START of the next" pace, not a guaranteed rate.
EXT_TOF_POLL_INTERVAL_SEC = 0.2

# djitellopy's own per-command timeout/retry can already stall for
# several seconds internally; cap how long a single query is allowed
# to run before this driver gives up on that attempt and starts the
# next one, so a truly wedged call can't block the poll loop forever.
EXT_TOF_QUERY_TIMEOUT_SEC = 8.0

_EXT_TOF_NUMBER_RE = re.compile(r"-?\d+(?:\.\d+)?")


def parse_ext_tof_response(response):
    """
        Parses the raw string returned by send_command_with_return("EXT tof?").
        Observed formats in the wild include a bare number ("153") and a
        prefixed form ("tof 153"); this extracts the first number present
        rather than assuming an exact format, and returns None for
        anything that doesn't contain one (e.g. "error", "", a timeout
        sentinel string djitellopy itself may return).

        Returns the parsed value in millimeters as a float, or None if
        response could not be parsed as a reading at all (a parse
        failure is NOT the same as an out-of-range reading - see
        is_ext_tof_valid() for range validity).
    """
    if not response:
        return None
    match = _EXT_TOF_NUMBER_RE.search(response)
    if not match:
        return None
    try:
        return float(match.group())
    except ValueError:
        return None


def is_ext_tof_valid(distance_mm):
    """
        True if distance_mm is a real, usable ground-distance reading -
        i.e. not the out-of-range sentinel band and not implausibly
        close. Mirrors perception.tof_scale_estimator.is_tof_valid()'s
        role for the built-in sensor.
    """
    if distance_mm is None:
        return False
    if distance_mm >= EXT_TOF_SENTINEL_MIN_MM:
        return False
    return EXT_TOF_MIN_VALID_MM < distance_mm <= EXT_TOF_MAX_VALID_MM


class ExtTofDriver:
    """
        Owns a dedicated background thread that repeatedly calls
        tello.send_command_with_return("EXT tof?"), parses the result,
        and exposes the latest reading plus a liveness/staleness flag.

        Does not start automatically - call start() once the
        TelloDriver this wraps is already connected (same lifecycle
        expectation as TelemetryMonitor/FrameReceiver/CommandHandler).
    """

    def __init__(self, tello_driver):
        """
            tello_driver: a drone_interface.tello_driver.TelloDriver
                instance, already connected. Reuses its .tello handle
                and .cmd_lock - this driver never opens its own
                connection or bypasses the shared command-channel lock.
        """
        self._tello_driver = tello_driver

        self._lock = threading.Lock()
        self._latest_distance_mm = None      # last PARSED value (valid or not), or None
        self._latest_valid_distance_mm = None  # last value that also passed is_ext_tof_valid()
        self._last_success_time = None       # monotonic time of last successful parse
        self._last_query_started = None      # monotonic time the in-flight query began

        self._thread = None
        self._shutdown = threading.Event()

    # ****************************************************************************************
    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._shutdown.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self, join_timeout=2.0):
        self._shutdown.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout)
    # ****************************************************************************************

    # ****************************************************************************************
    def _poll_loop(self):
        while not self._shutdown.is_set():
            self._last_query_started = time.monotonic()
            try:
                with self._tello_driver.cmd_lock:
                    response = self._tello_driver.tello.send_command_with_return("EXT tof?")
            except Exception as e:
                # A single failed/timed-out query must never kill the
                # poll loop - just try again next cycle. Liveness is
                # judged by _last_success_time, not by exceptions here.
                print(f"[ext_tof_driver] query failed (non-fatal): {e}")
                response = None

            distance_mm = parse_ext_tof_response(response) if response is not None else None

            with self._lock:
                if distance_mm is not None:
                    self._latest_distance_mm = distance_mm
                    self._last_success_time = time.monotonic()
                    if is_ext_tof_valid(distance_mm):
                        self._latest_valid_distance_mm = distance_mm

            self._shutdown.wait(timeout=EXT_TOF_POLL_INTERVAL_SEC)

    # ****************************************************************************************
    def read(self):
        """
            Returns a snapshot dict:
              {
                "distance_mm": float | None,        # last parsed value, valid or not
                "valid": bool,                        # last parsed value passed range check
                "is_stale": bool,                     # no successful parse recently enough
                "seconds_since_success": float | None,
              }

            "is_stale" covers both the documented multi-second hang
            case and a genuinely dead/disconnected sensor - callers
            (the scale estimator) should treat a stale reading as "this
            source is currently unavailable," not attempt to use
            distance_mm anyway.
        """
        with self._lock:
            distance_mm = self._latest_distance_mm
            last_success = self._last_success_time

        now = time.monotonic()
        seconds_since_success = (now - last_success) if last_success is not None else None
        is_stale = (
            seconds_since_success is None
            or seconds_since_success > EXT_TOF_STALE_AFTER_SEC
        )

        return {
            "distance_mm": distance_mm,
            "valid": (not is_stale) and is_ext_tof_valid(distance_mm),
            "is_stale": is_stale,
            "seconds_since_success": seconds_since_success,
        }
    # ****************************************************************************************
