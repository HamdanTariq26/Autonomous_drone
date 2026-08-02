#!/usr/bin/env python3
"""
scripts/validate_tof_scale.py

Validates perception/tof_scale_estimator.py against ground truth, isolated
from Depth-Anything-V2 and from flight control -- same rationale as
scripts/calibrate_scale_factor.py: hold the drone by hand, no motors, no
PID, so any error you see here comes from ToF + SLAM's own height
estimate, not from anything else in the stack.

THREE INDEPENDENT CHECKS, RUN IN ORDER
----------------------------------------
1. RAW SENSOR CHECK -- is the ToF hardware itself accurate?
   Hold the drone at a few heights you measure yourself (tape measure
   against the floor) and compare against the raw TOPIC_TOF_HEIGHT_CM
   reading. This has nothing to do with SLAM or scale factors at all --
   purely "does the sensor agree with a tape measure." If this fails,
   nothing downstream can be trusted; the problem is hardware/mounting,
   not software.

2. SELF-CONSISTENCY CHECK -- does the live scaled pose
   (current_pose_metric) agree with the raw ToF sensor's OWN reported
   delta, independent of any tape measure? Move the drone vertically; the
   script compares (end_tof_cm - start_tof_cm)/100 against the
   SLAM-scaled pose's height delta. If "tof" is the active scale source
   and working correctly, these two numbers should already be very close
   -- the estimator is *built from* this exact ratio. This check mostly
   validates the plumbing (is "tof" actually active, is the estimate
   reliable, is live_scaler applying it correctly), not sensor accuracy.

3. GROUND-TRUTH DISTANCE CHECK -- the same free-form test as
   calibrate_scale_factor.py (any direction, tape-measured distance), with
   a warning if "tof" isn't confirmed as the active scale source, so you
   know you're validating ToF+SLAM specifically and not whichever source
   "auto" happened to pick.

BEFORE RUNNING
--------------
Switch to ToF-only mode first: press 'm' in the manual control window
until the on-screen status reads "SCALE SRC: TOF". This script cannot
force that mode itself -- it only observes.

OPTIONAL PATCH (recommended, not required)
-------------------------------------------
This script can't see which scale source is currently active from outside
the process unless something publishes it. Add this to
perception/scale_factor_manager.py:

  # In __init__, after self._topics = TopicManager(self):
  from std_msgs.msg import String
  self._debug_pub = self.create_publisher(String, "/tello_autonomy/scale_source_debug", 10)

  # In _resolve_active_scale(), right after `if chosen is not None:`:
  dbg = String()
  dbg.data = f"map_id={map_id} mode={mode} tof_reliable={tof_reliable}"
  self._debug_pub.publish(dbg)

Without this patch every check below still runs -- the script just can't
auto-confirm which source is active, and asks you to confirm by eye
against the manual_control on-screen status text instead.
"""

import math
import os
import statistics
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_TELLO_AUTONOMY_ROOT = os.path.dirname(_THIS_DIR)
if _TELLO_AUTONOMY_ROOT not in sys.path:
    sys.path.insert(0, _TELLO_AUTONOMY_ROOT)

from config import constants  # noqa: E402
from perception.tof_scale_estimator import is_tof_valid  # noqa: E402 (reuse the real validity check)

SCALE_SOURCE_DEBUG_TOPIC = "/tello_autonomy/scale_source_debug"  # only populated if the optional patch is applied


class ToFValidator(Node):
    def __init__(self):
        super().__init__("tof_scale_validator")

        self._latest_pose = None
        self._latest_tof_cm = None
        self._latest_baro_m = None
        self._latest_source_debug = None

        self._pose_ready_event = threading.Event()
        self._tof_ready_event = threading.Event()

        self.create_subscription(
            PoseStamped, constants.TOPIC_CURRENT_POSE_METRIC, self._on_pose, 10
        )
        self.create_subscription(
            Float64, constants.TOPIC_TOF_HEIGHT_CM, self._on_tof, 10
        )
        self.create_subscription(
            Float64, constants.TOPIC_BARO_HEIGHT_M, self._on_baro, 10
        )
        self.create_subscription(
            String, SCALE_SOURCE_DEBUG_TOPIC, self._on_source_debug, 10
        )

    def _on_pose(self, msg: PoseStamped):
        self._latest_pose = msg.pose.position
        self._pose_ready_event.set()

    def _on_tof(self, msg: Float64):
        self._latest_tof_cm = msg.data
        self._tof_ready_event.set()

    def _on_baro(self, msg: Float64):
        self._latest_baro_m = msg.data

    def _on_source_debug(self, msg: String):
        self._latest_source_debug = msg.data

    def get_pose_xyz(self):
        p = self._latest_pose
        return None if p is None else (p.x, p.y, p.z)

    def sample_tof_cm(self, num_samples=5, delay_sec=0.1):
        """Average a few readings rather than trusting a single noisy tick."""
        samples = []
        for _ in range(num_samples):
            if self._latest_tof_cm is not None:
                samples.append(self._latest_tof_cm)
            self._pose_ready_event.wait(delay_sec)  # reuse as a cheap sleep-with-shutdown-awareness
        return statistics.mean(samples) if samples else None


def spin_in_background(node):
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return thread


def wait_for_ready(node, timeout_sec=60.0):
    print("Waiting for live pose + ToF data...")
    pose_ok = node._pose_ready_event.wait(timeout=timeout_sec)
    tof_ok = node._tof_ready_event.wait(timeout=1.0)  # already have pose event's wait behind us

    if not pose_ok:
        print(
            f"\nTimed out after {timeout_sec:.0f}s waiting for {constants.TOPIC_CURRENT_POSE_METRIC}. "
            "Check that main.py is running, SLAM has ACKed, and a scale factor has been computed."
        )
        return False
    if not tof_ok:
        print(
            f"\nNo messages seen yet on {constants.TOPIC_TOF_HEIGHT_CM}. "
            "Check that middleware/telemetry_bridge.py is running and the drone is connected."
        )
        return False

    print("Live pose and ToF data confirmed.\n")

    if node._latest_source_debug is not None:
        print(f"Active scale source (reported): {node._latest_source_debug}")
        if "mode=tof" not in node._latest_source_debug:
            print(
                "  WARNING: active mode does not report 'tof'. Press 'm' in the manual "
                "control window until the overlay reads 'SCALE SRC: TOF', then re-run."
            )
    else:
        print(
            "No scale-source debug topic seen (optional patch not applied -- see this "
            "script's docstring). CONFIRM BY EYE that manual_control's on-screen status "
            "reads 'SCALE SRC: TOF' before trusting checks 2 and 3 below."
        )
    return True


# ----------------------------------------------------------------------
# Check 1: raw sensor vs tape measure
# ----------------------------------------------------------------------
def run_raw_sensor_check(node):
    print("\n" + "=" * 60)
    print(" CHECK 1 - RAW ToF SENSOR vs TAPE MEASURE")
    print("=" * 60)
    print(
        "Hold the drone at a height you can measure with a tape measure "
        "(floor to the bottom of the drone). Enter 'done' instead of a "
        "height to move on to Check 2.\n"
    )

    errors = []
    while True:
        entry = input("True height above floor, in cm (or 'done'): ").strip()
        if entry.lower() == "done":
            break
        try:
            true_cm = float(entry)
        except ValueError:
            print("Not a number, try again.")
            continue

        tof_cm = node.sample_tof_cm()
        if tof_cm is None:
            print("No ToF reading available right now -- try again.")
            continue

        if not is_tof_valid(tof_cm):
            print(
                f"  Raw ToF = {tof_cm:.1f}cm is OUTSIDE the valid range "
                f"({constants.TOF_MIN_VALID_CM}-{constants.TOF_MAX_VALID_CM}cm) -- "
                "near-field clamp or out-of-range. Try a height inside that band."
            )
            continue

        error_cm = tof_cm - true_cm
        error_pct = (error_cm / true_cm) * 100.0 if true_cm > 0 else float("nan")
        errors.append(error_cm)
        print(f"  Raw ToF = {tof_cm:.1f}cm   True = {true_cm:.1f}cm   Error = {error_cm:+.1f}cm ({error_pct:+.1f}%)\n")

    if errors:
        print(f"\nCheck 1 summary: {len(errors)} sample(s), mean error = {statistics.mean(errors):+.1f}cm")
        if len(errors) > 1:
            print(f"  std_dev = {statistics.stdev(errors):.1f}cm")
    else:
        print("\nCheck 1: no samples collected.")


# ----------------------------------------------------------------------
# Check 2: self-consistency (raw ToF delta vs scaled-pose delta)
# ----------------------------------------------------------------------
def run_self_consistency_check(node):
    print("\n" + "=" * 60)
    print(" CHECK 2 - SELF-CONSISTENCY (raw ToF delta vs SLAM-scaled delta)")
    print("=" * 60)
    print(
        "No tape measure needed for this one. Hold the drone steady, mark "
        "START, move it vertically (up or down) by any amount you like, "
        "then mark END. The script compares the raw ToF sensor's own "
        "height change against the SLAM-scaled pose's height change.\n"
    )

    input("Press ENTER to mark START...")
    start_pose = node.get_pose_xyz()
    start_tof = node.sample_tof_cm()
    if start_pose is None or start_tof is None:
        print("Missing pose or ToF data -- aborting this check.")
        return
    print(f"  START: pose_y={start_pose[1]:.3f}m   raw_tof={start_tof:.1f}cm")

    input("\nMove the drone vertically, then press ENTER to mark END...")
    end_pose = node.get_pose_xyz()
    end_tof = node.sample_tof_cm()
    if end_pose is None or end_tof is None:
        print("Missing pose or ToF data -- aborting this check.")
        return
    print(f"  END:   pose_y={end_pose[1]:.3f}m   raw_tof={end_tof:.1f}cm")

    if not (is_tof_valid(start_tof) and is_tof_valid(end_tof)):
        print(
            "\n  One or both ToF readings are outside the valid range - this check "
            "needs both endpoints inside "
            f"({constants.TOF_MIN_VALID_CM}-{constants.TOF_MAX_VALID_CM}cm). Skipping."
        )
        return

    delta_pose_m = abs(end_pose[1] - start_pose[1])  # Y = Down, per trajectory_tracker.py convention
    delta_tof_m = abs(end_tof - start_tof) / 100.0

    diff_m = delta_pose_m - delta_tof_m
    diff_pct = (diff_m / delta_tof_m) * 100.0 if delta_tof_m > 1e-6 else float("nan")

    print(f"\n  SLAM-scaled height change:  {delta_pose_m:.3f} m")
    print(f"  Raw ToF height change:      {delta_tof_m:.3f} m")
    print(f"  Difference:                 {diff_m:+.3f} m ({diff_pct:+.1f}%)")

    if abs(diff_pct) < 5.0:
        print("  -> Consistent: the scaled pose is tracking the raw ToF sensor well.")
    else:
        print(
            "  -> Inconsistent: check that 'tof' is really the active source (Check "
            "above), that the estimator reports reliable=True, and that enough time "
            "has passed for its rolling window to fill."
        )


# ----------------------------------------------------------------------
# Check 3: ground-truth 3D distance (same style as calibrate_scale_factor.py)
# ----------------------------------------------------------------------
def run_ground_truth_check(node):
    print("\n" + "=" * 60)
    print(" CHECK 3 - GROUND-TRUTH DISTANCE (any direction, tape-measured)")
    print("=" * 60)
    print(
        "Move the drone a distance you can measure exactly (tape measure, "
        "any direction). This tests the full pipeline: SLAM shape + "
        "whichever scale source is currently active.\n"
    )

    input("Press ENTER to mark START...")
    start = node.get_pose_xyz()
    if start is None:
        print("No pose available -- aborting.")
        return
    print(f"  START: x={start[0]:.3f}  y={start[1]:.3f}  z={start[2]:.3f}")

    input("\nMove the drone, then press ENTER to mark END...")
    end = node.get_pose_xyz()
    if end is None:
        print("No pose available -- aborting.")
        return
    print(f"  END:   x={end[0]:.3f}  y={end[1]:.3f}  z={end[2]:.3f}")

    dx, dy, dz = end[0] - start[0], end[1] - start[1], end[2] - start[2]
    measured_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    print(f"\n  SLAM-scaled displacement: {measured_dist:.3f} m  (dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f})")

    while True:
        try:
            real_dist = float(input("\nEnter the REAL distance moved, in meters: ").strip())
            if real_dist <= 0:
                print("Must be positive.")
                continue
            break
        except ValueError:
            print("Not a valid number, try again.")

    error_abs = measured_dist - real_dist
    error_pct = (error_abs / real_dist) * 100.0

    print(f"\n  Real distance:      {real_dist:.3f} m")
    print(f"  SLAM-reported:      {measured_dist:.3f} m")
    print(f"  Absolute error:     {error_abs:+.3f} m")
    print(f"  Percent error:      {error_pct:+.1f} %")

    if node._latest_source_debug is not None and "mode=tof" not in node._latest_source_debug:
        print(
            "\n  NOTE: active mode did not report 'tof' - this result may reflect "
            "Depth-Anything or a mix, not ToF+SLAM specifically."
        )

    if abs(error_pct) < 5.0:
        print("\n  -> Within 5%: ToF+SLAM scale looks accurate.")
    elif abs(error_pct) < 15.0:
        print("\n  -> 5-15%: usable but worth another pass, ideally in 'tof'-only mode.")
    else:
        print("\n  -> >15%: significantly off - check Checks 1 and 2 above for where it breaks down.")


def main():
    rclpy.init()
    node = ToFValidator()
    spin_in_background(node)

    if not wait_for_ready(node):
        rclpy.shutdown()
        return

    run_raw_sensor_check(node)
    run_self_consistency_check(node)
    run_ground_truth_check(node)

    print("\nDone. Run any of these again (different heights/directions) before")
    print("drawing conclusions - a single sample can be noisy.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
