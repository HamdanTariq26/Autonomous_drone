"""
perception/tof_scale_estimator.py

Computes a candidate metric scale factor per map_id using the drone's
downward ToF sensor as the ground-truth height reference, as an alternative
to Depth-Anything-V2's CPU inference.  Runs continuously at whatever rate
TOPIC_TOF_HEIGHT_CM arrives (middleware/telemetry_bridge.py, ~TOF_POLL_HZ)
— no waiting on a 30-60s CPU inference cycle.

This module uses VERTICAL MOTION (delta ToF vs delta SLAM Y) to estimate
scale, not absolute heights.  This avoids the "changing origin" problem
where ToF and SLAM Y do not share the same zero point.

    scale_factor = |ΔToF_height_m| / |ΔSLAM_Y|

SLAM's Y comes from TOPIC_CURRENT_POSE_RAW.pose.position.y — Y is
"down" in this project's SLAM convention (see goals/trajectory_tracker.py's
coordinate-frame docstring: X=Right, Y=Down, Z=Forward), and the SLAM
origin is roughly the takeoff point, so climbing makes y more NEGATIVE.

By using differences between two points in time, the unknown offsets cancel
out and the resulting scale is independent of where the drone is flying.

This module does NOT replace perception/scale_factor.py or
scale_factor_manager.py's Depth-Anything pipeline — it is a second,
independent, much cheaper estimator.  perception/scale_factor_manager.py
decides which source is "active" per constants.SCALE_SOURCE_MODE
("tof" / "depth" / "auto"); this module only ever produces candidates via
get_scale_estimate() and never writes to
ScaleFactorManager.current_scale_factors directly.

Validity / noise handling
-------------------------
- ToF readings outside the open interval (TOF_MIN_VALID_CM, TOF_MAX_VALID_CM)
  are dropped.  This excludes both the SDK's near-field clamp (observed ~10 cm
  output when the true distance is under ~30 cm) and the out-of-range sentinel
  (~8192).
- Scale samples are only computed when there has been enough vertical motion
  between two points in the history (e.g. ≥20 cm ToF change, ≥0.05 SLAM Y).
- A rolling window of the last TOF_SCALE_ROLLING_WINDOW valid scale samples
  is kept per map_id.  The published scale estimate is the MEDIAN of those
  scale-factor values — robust to single-sample spikes.
- reliable=True only once the window contains at least half of
  TOF_SCALE_ROLLING_WINDOW samples AND the raw ToF cm readings in that window
  have a standard deviation under TOF_SCALE_STDDEV_THRESHOLD_CM.  This is
  exactly what "auto" mode in ScaleFactorManager checks before trusting ToF
  over Depth-Anything.
- Barometer disagreement is logged as a warning ONLY — it never blocks or
  triggers a fallback by itself.  Indoor barometers drift from HVAC and door
  pressure changes by amounts similar to this threshold, so a disagreement is
  as likely to mean the barometer is wrong as the ToF.
"""

import statistics
import time
from collections import deque
from typing import Dict, Deque, Tuple

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from rclpy.node import Node

from config import constants
from middleware.topic_manager import TopicManager


def _parse_map_id_from_frame_id(frame_id):
    """Same encoding used by perception.live_scaler / scale_factor_manager."""
    if not frame_id.startswith(constants.SLAM_MAP_FRAME_ID_PREFIX):
        return None
    suffix = frame_id[len(constants.SLAM_MAP_FRAME_ID_PREFIX):]
    try:
        return int(suffix)
    except ValueError:
        return None


def is_tof_valid(tof_cm):
    """
        True if tof_cm falls inside the valid open interval
        (TOF_MIN_VALID_CM, TOF_MAX_VALID_CM) — i.e. it's a real, usable
        ground-distance reading, not a near-field clamp or out-of-range
        sentinel.
    """
    return constants.TOF_MIN_VALID_CM < tof_cm < constants.TOF_MAX_VALID_CM


class ToFScaleEstimator(Node):
    """
        Subscribes to raw SLAM pose (per map_id), raw ToF distance, and
        barometer height; maintains a rolling per-map_id window of
        ToF-derived scale-factor candidates using VERTICAL MOTION (deltas).

        Call get_scale_estimate(map_id) to read the current smoothed
        estimate and whether it is currently trustworthy.  Called by
        ScaleFactorManager's _resolve_active_scale() — never reads
        current_scale_factors directly.
    """

    def __init__(self, node_name="tof_scale_estimator"):
        super().__init__(node_name)
        self._topics = TopicManager(self)

        self._topics.get_subscription(
            constants.TOPIC_CURRENT_POSE_RAW, PoseStamped, self._on_pose_raw
        )
        self._topics.get_subscription(
            constants.TOPIC_TOF_HEIGHT_CM, Float64, self._on_tof
        )
        self._topics.get_subscription(
            constants.TOPIC_BARO_HEIGHT_M, Float64, self._on_baro
        )

        # Latest-value store: pose callback uses whatever arrived most
        # recently on the ToF/baro topics.  No exact time-sync needed —
        # at 10 Hz ToF vs ~30 Hz pose, skew is at most ~100 ms, which is
        # acceptable for a scale estimate that is already being median-filtered.
        self._latest_tof_cm = None
        self._latest_baro_m = None
        self._last_baro_warn_time = 0.0  # monotonic time of last barometer disagreement warning

        # map_id -> deque of (tof_cm, scale_factor) tuples, capped at the
        # most recent TOF_SCALE_ROLLING_WINDOW valid samples.
        self._windows: Dict[int, Deque[Tuple[float, float]]] = {}

        # map_id -> deque of (tof_m, slam_y, stamp) for delta-based scale estimation.
        # Each entry: (tof_m, slam_y, monotonic_timestamp)
        self._delta_history: Dict[int, Deque[Tuple[float, float, float]]] = {}

        self.get_logger().info(
            f"ToFScaleEstimator: ready. Valid ToF range "
            f"({constants.TOF_MIN_VALID_CM}, {constants.TOF_MAX_VALID_CM}) cm, "
            f"rolling window={constants.TOF_SCALE_ROLLING_WINDOW} samples, "
            f"delta-based scale estimation enabled."
        )

    # ****************************************************************************************
    def _on_tof(self, msg: Float64):
        self._latest_tof_cm = msg.data

    def _on_baro(self, msg: Float64):
        """
            Store barometer reading, but ignore obviously broken values.
            Indoor baro should be roughly 0–100 m above ground.
        """
        value_m = msg.data
        if not (0.0 <= value_m <= 100.0):
            # Ignore obviously broken baro readings (e.g. 66 km).
            self._latest_baro_m = None
            return
        self._latest_baro_m = value_m
    # ****************************************************************************************

    # ****************************************************************************************
    def _on_pose_raw(self, msg: PoseStamped):
        """
            For each incoming raw SLAM pose, update the delta-based scale estimate
            using vertical motion (ΔToF vs ΔSLAM Y).  Invalid ToF readings or
            insufficient motion are skipped — no sample stored.
        """
        map_id = _parse_map_id_from_frame_id(msg.header.frame_id)
        if map_id is None:
            return

        if self._latest_tof_cm is None:
            return  # telemetry_bridge hasn't published a ToF reading yet

        tof_cm = self._latest_tof_cm
        if not is_tof_valid(tof_cm):
            return  # near-field clamp or out-of-range sentinel — not usable

        tof_m = tof_cm / 100.0
        slam_y = msg.pose.position.y  # Y is "down" in SLAM convention
        stamp = time.monotonic()

        # Delegate to delta-based estimator.
        self._update_scale_with_deltas(map_id, tof_m, slam_y, stamp)

    # ****************************************************************************************
    def _update_scale_with_deltas(
        self,
        map_id: int,
        tof_m: float,
        slam_y: float,
        stamp: float,
    ) -> None:
        """
            Update scale estimate for map_id using vertical motion (delta ToF vs delta SLAM Y).

            We only compute a scale sample when:
              - We have two points in the history with enough vertical motion between them.
              - Both ToF and SLAM Y changes are large enough to be above noise.
              - The resulting scale is within a sane range.

            This avoids the 'changing origin' problem of using absolute heights.
        """
        # Parameters (tune if needed)
        MIN_DELTA_TOF_M = 0.20       # 20 cm minimum vertical motion in ToF
        MIN_DELTA_SLAM_Y = 0.05      # minimum SLAM Y motion (arbitrary units)
        MAX_HISTORY_AGE_SEC = 3.0    # ignore points older than this

        hist = self._delta_history.setdefault(map_id, deque(maxlen=30))

        # Clean old points
        now = stamp
        while hist and (now - hist[0][2]) > MAX_HISTORY_AGE_SEC:
            hist.popleft()

        # Need at least one older point to form a delta
        if len(hist) < 1:
            hist.append((tof_m, slam_y, now))
            return

        # Use the oldest point in the window as the base for delta
        base_tof, base_slam_y, base_stamp = hist[0]

        delta_tof = tof_m - base_tof
        delta_slam_y = slam_y - base_slam_y

        # If not enough motion yet, just append and wait for more movement
        if abs(delta_tof) < MIN_DELTA_TOF_M or abs(delta_slam_y) < MIN_DELTA_SLAM_Y:
            hist.append((tof_m, slam_y, now))
            return

        # Compute scale from the ratio of deltas
        scale = abs(delta_tof / delta_slam_y)

        # Sanity bounds (reuse your existing constants)
        if not (constants.SCALE_RATIO_MIN <= scale <= constants.SCALE_RATIO_MAX):
            # Out of plausible range; discard this sample.
            # Still append current point so future deltas can use it.
            hist.append((tof_m, slam_y, now))
            return

        # Append the new point
        hist.append((tof_m, slam_y, now))

        # Treat this 'scale' as a candidate sample, similar to before.
        # Store it in the existing rolling window structure so the rest
        # of your pipeline (get_scale_estimate, quarantine, etc.) can stay the same.
        window = self._windows.setdefault(
            map_id, deque(maxlen=constants.TOF_SCALE_ROLLING_WINDOW)
        )
        # We store (tof_cm, scale) as before, but scale is now delta-based.
        window.append((tof_m * 100.0, scale))

        # Barometer disagreement warning (if baro is valid)
        self._maybe_warn_baro_disagreement(tof_m * 100.0)

    # ****************************************************************************************
    def _maybe_warn_baro_disagreement(self, tof_cm: float):
        """
            Logs a warning when ToF and barometer disagree by more than
            BARO_TOF_DISAGREEMENT_WARN_CM.  Throttled to at most once every
            BARO_WARN_THROTTLE_SEC seconds.
        """
        if self._latest_baro_m is None:
            return  # no valid baro reading

        tof_m = tof_cm / 100.0
        disagreement = abs(tof_m - self._latest_baro_m)

        if disagreement <= constants.BARO_TOF_DISAGREEMENT_WARN_CM / 100.0:
            return  # within tolerance

        now = time.monotonic()
        if (now - self._last_baro_warn_time) < constants.BARO_WARN_THROTTLE_SEC:
            return  # throttle: skip this warning

        self.get_logger().warn(
            f"ToF and barometer disagree by {disagreement * 100.0:.1f} cm "
            f"(ToF={tof_cm:.1f} cm, baro={self._latest_baro_m * 100.0:.1f} cm). "
            "Check sensor health and mounting."
        )
        self._last_baro_warn_time = now
    # *****************************************************************:***********************

    # ****************************************************************************************
    def get_scale_estimate(self, map_id: int):
        """
            Returns (scale_factor | None, reliable: bool) for the given map_id.

            scale_factor is the median of the rolling window's scale-factor
            samples.  reliable is True only when:
              1. The window contains at least half of TOF_SCALE_ROLLING_WINDOW
                 samples (i.e. data has been accumulating long enough to trust
                 the median), AND
              2. The raw ToF cm readings in that window have a standard deviation
                 at or below TOF_SCALE_STDDEV_THRESHOLD_CM (stable readings).

            Called by ScaleFactorManager._resolve_active_scale() to decide
            whether to trust ToF over Depth-Anything in "auto" mode.
        """
        window = self._windows.get(map_id)
        if not window:
            return None, False

        min_samples = max(3, constants.TOF_SCALE_ROLLING_WINDOW // 2)
        if len(window) < min_samples:
            return None, False

        tof_values = [t for t, _s in window]
        scale_values = [s for _t, s in window]

        scale_estimate = statistics.median(scale_values)
        tof_stddev = statistics.stdev(tof_values) if len(tof_values) > 1 else 0.0
        reliable = tof_stddev <= constants.TOF_SCALE_STDDEV_THRESHOLD_CM

        return scale_estimate, reliable
    # ****************************************************************************************

    # ****************************************************************************************
    def destroy_node(self):
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************
