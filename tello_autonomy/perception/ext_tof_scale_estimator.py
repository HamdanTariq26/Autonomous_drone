"""
perception/ext_tof_scale_estimator.py

Computes a candidate metric scale factor per map_id from the Tello
Extension Kit's downward ToF sensor (drone_interface.ext_tof_driver,
"EXT tof?", +/-4mm accurate up to ~1.2m), as a HIGHER-PRIORITY
alternative to perception/tof_scale_estimator.py's built-in-sensor
estimate and perception/scale_factor_manager.py's Depth-Anything
pipeline. See that module's docstring for the shared design rationale
(delta-based estimation, avoiding the "changing origin" problem,
median smoothing) - this module follows the same shape, upgraded with
a proper least-squares fit instead of a single pairwise ratio, since
the extension-kit sensor's much better precision (+/-4mm vs the
built-in sensor's cm-level noise) makes that upgrade worth it here.

This module contains TWO things:
  1. ExtTofScaleEstimator - pure Python math (no ROS2 dependency),
     the exact same role perception/tof_scale_estimator.py's own math
     plays internally. Kept import-testable without rclpy, same as
     every other pure-math module in perception/ (scale_factor.py,
     depth_backprojection.py, pose_transform.py).
  2. ExtTofScaleEstimatorNode - a thin rclpy.node.Node wrapper around
     (1), following the EXACT same shape as
     perception.tof_scale_estimator.ToFScaleEstimator: owns its own
     TOPIC_CURRENT_POSE_RAW subscription (map_id + slam_y come from
     there) plus the ext-tof bridge's two topics
     (TOPIC_EXT_TOF_DISTANCE_MM / TOPIC_EXT_TOF_VALID), and exposes
     get_scale_estimate(map_id) for ScaleFactorManager to poll -
     exactly like ToFScaleEstimator.get_scale_estimate() already
     works. This keeps the "one estimator = one Node, ScaleFactorManager
     only ever reads a plain accessor method" pattern consistent
     across both ToF sources, rather than ScaleFactorManager itself
     taking on a new raw-pose subscription it doesn't otherwise need.

WHY LEAST SQUARES, NOT A SINGLE RATIO
---------------------------------------
tof_scale_estimator.py computes each sample as a single pairwise ratio
|delta_tof| / |delta_slam_y| between two points in time. That's fine
when many independent samples get median-filtered together (noise
mostly cancels in the median), but it is fragile per-sample: if
delta_slam_y happens to be small for that particular pair, the ratio
is dominated by noise or can blow up, and a single such sample can
distort the window before the median has enough points to drown it
out.

The standard fix (this is how scale recovery is done in the VO/SLAM
literature when you have a set of paired measurements that should
satisfy a single linear relationship, e.g. "metric quantity ~= s *
slam quantity" for every pair in a window) is total-least-squares /
ordinary-least-squares regression through the origin: given N pairs
(x_i, y_i) = (slam_delta_i, tof_delta_i) that should satisfy
y_i = s * x_i for the single unknown scalar s, the least-squares
estimate of s minimizing sum((y_i - s*x_i)^2) is the closed form

    s = sum(x_i * y_i) / sum(x_i^2)

This uses EVERY pair in the window simultaneously rather than each
pair alone producing its own independent ratio, so noise in any one
pair contributes proportionally less to the final estimate (weighted
by how much SLAM motion that pair actually represents, x_i^2) instead
of contributing an equal, unweighted "vote" the way a per-pair-ratio
median does. This is the "correct calculations, not just simple
ratios" the person asked for.

WHY THE FIT WINDOW USES BASE-ANCHORED DELTAS
------------------------------------------------
Like tof_scale_estimator.py, deltas are taken against a single
"base" anchor point (the oldest sample still within
EXT_TOF_HISTORY_MAX_AGE_SEC) rather than consecutive-frame deltas.
Consecutive-frame deltas at high pose-callback rates are dominated by
sensor jitter (tiny motion, large relative noise); anchoring against
an older base point ensures each delta represents enough real motion
for the ratio it contributes to be meaningful. Once accumulated,
though, EVERY (base, sample) pair still in the rolling window
participates in the single LS fit above - this is different from
tof_scale_estimator.py, which discards the window down to one
ratio-per-accepted-delta-step; here we keep the underlying (x, y)
pairs and re-fit over the whole live set each time, which is what
lets the LS formula do its job across multiple motions instead of
just smoothing a stream of already-lossy individual ratios.

WHY THE OUTPUT DOESN'T JUMP EVERY TICK
------------------------------------------
_current_scale_by_map holds the actively-published estimate for each
map_id. A freshly computed LS fit only overwrites it once:
  1. the window has accumulated enough distinct base-anchored pairs
     (EXT_TOF_MIN_FIT_SAMPLES) to make the fit meaningful, AND
  2. the new fit differs from the currently-published value by more
     than EXT_TOF_UPDATE_DEADBAND_RATIO (a small relative deadband) -
     tiny fit-to-fit wobble from ordinary sensor noise is absorbed
     silently rather than republished as a "new" scale factor, AND
  3. a minimum EXT_TOF_MIN_UPDATE_INTERVAL_SEC has elapsed since the
     last accepted update - an additional hard rate limit so this
     estimator can never update on every single pose tick even if (1)
     and (2) both happen to pass on consecutive callbacks.
get_scale_estimate() always returns the last ACCEPTED value, not the
instantaneous fit - callers never see the raw per-tick fit output.

STALENESS / HANG HANDLING
---------------------------
Every incoming sample is tagged with the driver's own is_stale flag
(see ext_tof_driver.ExtTofDriver.read() - covers both the documented
multi-second command hang and a genuinely dead sensor). A stale
reading is never added to a map_id's history and never updates
reliability - get_scale_estimate() reports reliable=False once the
most recent sample age exceeds the same staleness threshold the
driver itself uses, so a hang naturally and automatically hands
priority back to whatever ScaleFactorManager falls back to (built-in
ToF / Depth-Anything) without this module needing to know anything
about arbitration - that policy lives entirely in
ScaleFactorManager._resolve_active_scale(), same as the existing
"tof"/"depth"/"auto" sources.

MAP_ID LIFECYCLE
-------------------
All state is keyed by map_id and reset on a new one - no history
carries across a SLAM re-init, matching every other per-map_id
estimator in this codebase (tof_scale_estimator.py,
scale_factor_manager.py's _map_state/_quarantine dicts, etc).
"""

import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64
from rclpy.node import Node

from config import constants
from middleware.topic_manager import TopicManager


# --- Validity / physical bounds (mm) - mirrors ext_tof_driver's own,
# duplicated here as float constants so this module has no import-time
# dependency on drone_interface (perception/ stays independent of
# drone_interface/, matching the existing layering in this codebase -
# e.g. perception/scale_factor.py never imports drone_interface either).
EXT_TOF_SENTINEL_MIN_MM = 8190.0
EXT_TOF_MAX_VALID_MM = 1150.0
EXT_TOF_MIN_VALID_MM = 20.0

# How long a base-anchor sample is kept before it's evicted and a new
# base is chosen. Mirrors tof_scale_estimator.py's MAX_HISTORY_AGE_SEC
# concept - bounds how "stale" the anchor point can be while still
# being used to form a delta.
EXT_TOF_HISTORY_MAX_AGE_SEC = 4.0

# Minimum motion (both in raw SLAM units and in real mm) required
# before a (base, sample) pair is added to the fit window. Too little
# motion means the pair is almost pure noise and would only add a
# near-zero-weight, high-relative-error term to the LS sum.
EXT_TOF_MIN_DELTA_SLAM_Y = 0.05
EXT_TOF_MIN_DELTA_MM = 15.0

# Rolling window of (slam_delta, tof_delta_mm) pairs kept per map_id
# for the LS fit. Bounded so old motion eventually ages out rather
# than a scale estimate from 5 minutes ago still influencing "now".
EXT_TOF_FIT_WINDOW_SIZE = 40

# Minimum number of pairs in the window before a fit is even attempted.
# Below this the LS estimate is too noisy to trust over whatever is
# currently active.
EXT_TOF_MIN_FIT_SAMPLES = 6

# A freshly computed fit only overwrites the published estimate if it
# differs from the current one by more than this fraction - absorbs
# ordinary fit-to-fit wobble so the published value doesn't visibly
# change on every recompute.
EXT_TOF_UPDATE_DEADBAND_RATIO = 0.03

# Hard rate limit: even if (1) enough samples and (2) outside the
# deadband both hold, don't publish a new value more often than this.
EXT_TOF_MIN_UPDATE_INTERVAL_SEC = 1.0

# A map_id's estimate is reported unreliable once the most recent
# sample is older than this - matches
# ext_tof_driver.EXT_TOF_STALE_AFTER_SEC's intent (covers the
# documented multi-second command hang) without importing that module.
EXT_TOF_SAMPLE_STALE_AFTER_SEC = 10.0


def is_ext_tof_reading_valid(distance_mm):
    if distance_mm is None:
        return False
    if distance_mm >= EXT_TOF_SENTINEL_MIN_MM:
        return False
    return EXT_TOF_MIN_VALID_MM < distance_mm <= EXT_TOF_MAX_VALID_MM


class ExtTofScaleEstimator:
    """
        Pure Python (no ROS2 dependency, mirrors the pure-math modules
        like perception/scale_factor.py) - feed it samples via
        add_sample(), read the current best estimate via
        get_scale_estimate(). ExtTofScaleEstimatorNode (below) is the
        ROS2-facing wrapper that owns the actual subscriptions and
        calls add_sample() on each update - this class itself has zero
        rclpy imports and can be unit-tested exactly the way the
        sanity checks for this module already do, without rclpy.init()
        ever running.
    """

    def __init__(self):
        # map_id -> deque[(slam_y, distance_mm, monotonic_time)] - the
        # base-anchored raw sample history, same shape as
        # tof_scale_estimator.py's _delta_history.
        self._history: Dict[int, Deque[Tuple[float, float, float]]] = {}

        # map_id -> deque[(slam_delta, tof_delta_mm)] - accumulated
        # pairs actually fed into the LS fit.
        self._fit_pairs: Dict[int, Deque[Tuple[float, float]]] = {}

        # map_id -> last ACCEPTED (published) scale factor (mm per raw
        # SLAM unit) - what get_scale_estimate() returns.
        self._published_scale: Dict[int, float] = {}

        # map_id -> monotonic time of the last accepted update - drives
        # the EXT_TOF_MIN_UPDATE_INTERVAL_SEC rate limit.
        self._last_update_time: Dict[int, float] = {}

        # map_id -> monotonic time of the most recent sample added
        # (valid or not) - drives the staleness/reliable() check
        # independent of whether that sample produced a new fit pair.
        self._last_sample_time: Dict[int, float] = {}

    # ****************************************************************************************
    def reset_map(self, map_id: int) -> None:
        """Drops all state for map_id - call on a SLAM re-init, same as
        ScaleFactorManager._stop_tracking() does for its own per-map_id dicts."""
        self._history.pop(map_id, None)
        self._fit_pairs.pop(map_id, None)
        self._published_scale.pop(map_id, None)
        self._last_update_time.pop(map_id, None)
        self._last_sample_time.pop(map_id, None)

    # ****************************************************************************************
    def add_sample(
        self,
        map_id: int,
        slam_y: float,
        distance_mm: Optional[float],
        is_stale: bool,
        now: Optional[float] = None,
    ) -> None:
        """
            Feed one (SLAM raw pose, ext-tof reading) observation for
            map_id. Call this once per raw-pose update that has a
            reasonably time-aligned ext-tof reading available (exact
            sub-tick sync is not required - see
            perception/tof_scale_estimator.py's own note on this: at
            these rates, skew is small relative to what the median/LS
            smoothing already absorbs).

            slam_y: the drone's raw SLAM Y position (down-axis, same
                convention as tof_scale_estimator.py - see
                goals/trajectory_tracker.py's coordinate-frame
                docstring).
            distance_mm: latest parsed ext-tof reading in mm, or None
                if nothing has been parsed yet at all.
            is_stale: the ext-tof driver's own staleness flag (covers
                both "no data parsed recently" and the documented
                multi-second command hang) - a stale sample is
                recorded for reliability tracking but never
                contributes a fit pair.
            now: monotonic time override, for testing. Defaults to
                time.monotonic().
        """
        now = now if now is not None else time.monotonic()

        if not is_stale and distance_mm is not None:
            self._last_sample_time[map_id] = now

        if is_stale or not is_ext_tof_reading_valid(distance_mm):
            # Still worth keeping the history deque's age-eviction
            # running so a long gap doesn't leave a stale anchor sitting
            # around forever, but there is nothing usable to add.
            self._evict_old_history(map_id, now)
            return

        hist = self._history.setdefault(map_id, deque(maxlen=200))
        self._evict_old_history(map_id, now)

        if len(hist) == 0:
            hist.append((slam_y, distance_mm, now))
            return

        base_slam_y, base_distance_mm, _base_time = hist[0]
        delta_slam = slam_y - base_slam_y
        delta_mm = distance_mm - base_distance_mm

        if abs(delta_slam) < EXT_TOF_MIN_DELTA_SLAM_Y or abs(delta_mm) < EXT_TOF_MIN_DELTA_MM:
            hist.append((slam_y, distance_mm, now))
            return

        pairs = self._fit_pairs.setdefault(map_id, deque(maxlen=EXT_TOF_FIT_WINDOW_SIZE))
        pairs.append((delta_slam, delta_mm))

        hist.append((slam_y, distance_mm, now))

        self._maybe_refit(map_id, now)

    def _evict_old_history(self, map_id: int, now: float) -> None:
        hist = self._history.get(map_id)
        if not hist:
            return
        while hist and (now - hist[0][2]) > EXT_TOF_HISTORY_MAX_AGE_SEC:
            hist.popleft()

    # ****************************************************************************************
    def _maybe_refit(self, map_id: int, now: float) -> None:
        """
            Recomputes the least-squares scale fit over the current
            window and decides whether to accept it as the new
            published value, per the three gates described in the
            module docstring (enough samples, outside the deadband,
            minimum update interval elapsed).
        """
        pairs = self._fit_pairs.get(map_id)
        if pairs is None or len(pairs) < EXT_TOF_MIN_FIT_SAMPLES:
            return

        fitted = self._least_squares_scale(pairs)
        if fitted is None:
            return

        current = self._published_scale.get(map_id)
        if current is None:
            # First-ever fit for this map_id - nothing to compare
            # against, adopt outright (mirrors ScaleFactorManager.
            # _maybe_adopt_scale()'s "no active value yet" case).
            self._published_scale[map_id] = fitted
            self._last_update_time[map_id] = now
            return

        last_update = self._last_update_time.get(map_id, 0.0)
        if (now - last_update) < EXT_TOF_MIN_UPDATE_INTERVAL_SEC:
            return  # hard rate limit - never update more often than this

        if current == 0.0:
            relative_change = float("inf")
        else:
            relative_change = abs(fitted - current) / abs(current)

        if relative_change <= EXT_TOF_UPDATE_DEADBAND_RATIO:
            return  # within the deadband - ordinary fit wobble, don't republish

        self._published_scale[map_id] = fitted
        self._last_update_time[map_id] = now

    @staticmethod
    def _least_squares_scale(pairs) -> Optional[float]:
        """
            Closed-form least-squares fit through the origin:
            s = sum(x_i * y_i) / sum(x_i^2), where x_i = slam_delta_i,
            y_i = tof_delta_mm_i. See the module docstring for the
            derivation/rationale. Returns None if the denominator is
            degenerate (all slam deltas ~zero - should not happen given
            the EXT_TOF_MIN_DELTA_SLAM_Y gate in add_sample(), but
            guarded here defensively).
        """
        sum_xy = 0.0
        sum_xx = 0.0
        for slam_delta, tof_delta_mm in pairs:
            sum_xy += slam_delta * tof_delta_mm
            sum_xx += slam_delta * slam_delta

        if sum_xx <= 1e-9:
            return None

        return sum_xy / sum_xx

    # ****************************************************************************************
    def get_scale_estimate(self, map_id: int, now: Optional[float] = None):
        """
            Returns (scale_mm_per_slam_unit | None, reliable: bool).

            IMPORTANT UNIT NOTE: this returns mm per raw-SLAM-unit, to
            match the sensor's own native mm output (the person
            explicitly asked for this to stay in mm rather than being
            silently converted). ScaleFactorManager's
            current_scale_factors / everything downstream of it
            (LiveScaler, live-point scaling, etc) expects METERS per
            raw-SLAM-unit, matching Depth-Anything's and the built-in
            ToF estimator's convention - so the caller MUST divide this
            value by 1000.0 before writing it into
            current_scale_factors. See
            ScaleFactorManager._resolve_active_scale() for where that
            conversion happens; kept there rather than here so this
            module's return value stays a direct, inspectable
            reflection of the sensor's own native mm readings.

            reliable is True only when a genuinely fresh, non-stale
            sample has been seen recently (within
            EXT_TOF_SAMPLE_STALE_AFTER_SEC) AND a published estimate
            actually exists. Once the sensor goes stale (out of the
            ~1.2m range, or a hung query with nothing parsed recently),
            reliable flips to False immediately even though the last
            published scale value is still returned - callers must
            check reliable, not just whether a value is present, so
            they fall back to a lower-priority source instead of
            trusting a scale factor computed from ground the drone
            climbed away from minutes ago.
        """
        now = now if now is not None else time.monotonic()

        published = self._published_scale.get(map_id)
        if published is None:
            return None, False

        last_sample = self._last_sample_time.get(map_id)
        if last_sample is None or (now - last_sample) > EXT_TOF_SAMPLE_STALE_AFTER_SEC:
            return published, False

        return published, True


class ExtTofScaleEstimatorNode(Node):
    """
        ROS2-facing wrapper around ExtTofScaleEstimator, matching
        perception.tof_scale_estimator.ToFScaleEstimator's shape
        exactly:
          - Subscribes to TOPIC_CURRENT_POSE_RAW itself (map_id and
            slam_y both come from there - same source ToFScaleEstimator
            already reads).
          - Subscribes to the ext-tof bridge's two topics
            (middleware.ext_tof_bridge.ExtTofBridge) and caches the
            latest distance/validity, the same "latest-value store, no
            exact time-sync needed" pattern ToFScaleEstimator uses for
            its own ToF/baro topics (see that module's docstring for
            why: skew between ~10Hz sensor topics and ~30Hz pose is
            small relative to what the delta/LS smoothing already
            absorbs).
          - Every raw pose callback feeds the estimator via
            add_sample() - this is the ONLY place add_sample() is ever
            called; the underlying math never sees rclpy at all.
          - get_scale_estimate(map_id) delegates straight to the
            wrapped estimator - ScaleFactorManager calls this exactly
            like it already calls self._tof_estimator.get_scale_estimate(map_id).

        Construction/lifecycle mirrors ToFScaleEstimator too: built and
        spun as its own NodeRunner in scripts/main.py, handed to
        ScaleFactorManager's constructor as a plain object reference
        (ScaleFactorManager reads it via a method call, never via a
        topic - same "same-process direct reference" reasoning
        perception/live_scaler.py's own docstring gives for why it
        holds a ScaleFactorManager reference instead of a service
        round-trip).
    """

    def __init__(self, node_name="ext_tof_scale_estimator"):
        super().__init__(node_name)
        self._topics = TopicManager(self)

        self._estimator = ExtTofScaleEstimator()

        # Latest-value store for the ext-tof bridge's topics - same
        # pattern as ToFScaleEstimator._latest_tof_cm /
        # _latest_baro_m. Updated independently of the pose callback;
        # the pose callback (at ~30Hz) is what actually drives
        # add_sample() using whichever of these arrived most recently.
        self._latest_distance_mm = None
        self._latest_valid = False

        self._topics.get_subscription(
            constants.TOPIC_CURRENT_POSE_RAW, PoseStamped, self._on_pose_raw
        )
        self._topics.get_subscription(
            constants.TOPIC_EXT_TOF_DISTANCE_MM, Float64, self._on_distance
        )
        self._topics.get_subscription(
            constants.TOPIC_EXT_TOF_VALID, Bool, self._on_valid
        )

        self.get_logger().info(
            f"ExtTofScaleEstimatorNode: ready. Subscribed to "
            f"{constants.TOPIC_CURRENT_POSE_RAW}, "
            f"{constants.TOPIC_EXT_TOF_DISTANCE_MM}, "
            f"{constants.TOPIC_EXT_TOF_VALID}."
        )

    # ****************************************************************************************
    def _on_distance(self, msg: Float64):
        self._latest_distance_mm = msg.data

    def _on_valid(self, msg: Bool):
        self._latest_valid = msg.data
    # ****************************************************************************************

    # ****************************************************************************************
    def _on_pose_raw(self, msg: PoseStamped):
        """
            Mirrors ToFScaleEstimator._on_pose_raw(): parses map_id
            from the frame_id encoding every raw-pose message carries
            (config.constants.SLAM_MAP_FRAME_ID_PREFIX), and feeds one
            sample into the wrapped estimator per callback. slam_y is
            msg.pose.position.y - Y is "down" in this project's SLAM
            convention (see goals/trajectory_tracker.py's docstring),
            same axis ToFScaleEstimator already uses.
        """
        map_id = _parse_map_id_from_frame_id(msg.header.frame_id)
        if map_id is None:
            return

        self._estimator.add_sample(
            map_id=map_id,
            slam_y=msg.pose.position.y,
            distance_mm=self._latest_distance_mm,
            is_stale=not self._latest_valid,
        )
    # ****************************************************************************************

    # ****************************************************************************************
    def get_scale_estimate(self, map_id: int):
        """Delegates to the wrapped ExtTofScaleEstimator - see its docstring
        for the (scale_mm_per_slam_unit | None, reliable) return shape and
        the mm-vs-meters unit note ScaleFactorManager must respect."""
        return self._estimator.get_scale_estimate(map_id)

    def reset_map(self, map_id: int):
        """Drops per-map_id state - call from ScaleFactorManager._stop_tracking(),
        same as self._tof_estimator would be reset if it exposed this
        (ToFScaleEstimator today just lets stale entries age out passively;
        this estimator supports an explicit reset since ScaleFactorManager
        already knows the exact moment a map_id goes stale)."""
        self._estimator.reset_map(map_id)
    # ****************************************************************************************

    # ****************************************************************************************
    def destroy_node(self):
        self._topics.destroy_all()
        super().destroy_node()
    # ****************************************************************************************


def _parse_map_id_from_frame_id(frame_id):
    """Same encoding used by perception.live_scaler / scale_factor_manager /
    perception.tof_scale_estimator - see any of those for details."""
    if not frame_id.startswith(constants.SLAM_MAP_FRAME_ID_PREFIX):
        return None
    suffix = frame_id[len(constants.SLAM_MAP_FRAME_ID_PREFIX):]
    try:
        return int(suffix)
    except ValueError:
        return None
