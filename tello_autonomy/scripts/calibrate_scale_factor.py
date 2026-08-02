#!/usr/bin/env python3
"""
scripts/calibrate_scale_factor.py

Standalone real-world scale-factor calibration/validation tool.

WHY THIS EXISTS
----------------
perception/scale_factor.py computes a metric scale factor by comparing
ORB-SLAM3's arbitrary-unit depth against Depth-Anything-V2's metric
depth predictions. That estimate can be off -- too few keyframes, a
biased checkpoint for your specific lens/scene, motion blur, etc. This
script gives you a concrete, physical ground-truth number: move the
drone exactly 2.000m (or whatever you measure with a tape measure) and
see exactly how far off current_pose_metric thinks you moved.

Doing this by hand (not flying) also isolates the two problems you're
debugging from each other: holding the drone removes the flight
controller/PID entirely from the loop, so any distance error you see
here is coming from SLAM + scale, not from Tello drift or your P-gains.
If THIS test comes back accurate but the drone still drifts in flight,
that points you back at trajectory_tracker.py / the P-gains instead.

HOW SCALE-READINESS IS DETECTED
--------------------------------
No polling, no guessing: perception/live_scaler.py only ever publishes
on TOPIC_CURRENT_POSE_METRIC once a scale factor exists for the
current map_id -- it silently drops raw pose/points before that (see
LiveScaler._on_pose_raw's `if scale is None: return`). So the arrival
of the FIRST message on that topic IS the "scale factor is ready"
signal, by construction.

HOW TO USE
----------
1. Run this AFTER main.py is already running (drone connected, SLAM
   handshake done). The drone can be sitting in your hand with motors
   off -- this only needs live pose data, not flight.
2. It waits for the first current_pose_metric message, then tells you
   scale is ready.
3. Press ENTER to mark your START position (hold the drone steady).
4. Physically move the drone a distance YOU know exactly (tape
   measure recommended -- a straight 2m push works well).
5. Press ENTER again to mark the END position, then type in the real
   distance you moved it.
6. It prints measured distance, per-axis breakdown, absolute/percent
   error, and a suggested corrected scale factor.

This is read-only diagnostics -- it does not change your scale factor.
Run it a few times (different distances/directions) before concluding
anything; a single sample can be noisy.

OPTIONAL PATCH (recommended, not required)
-------------------------------------------
verify_occupancy_map.py already subscribes to a topic called
"/tello_autonomy/scale_factor" expecting a numeric Float64 -- but
nothing in the current codebase actually publishes it. If you add the
five lines below to perception/scale_factor_manager.py, this script
(and verify_occupancy_map.py) will also show you the exact current
scale factor number and a precise suggested replacement, instead of
just a correction ratio.

  # In ScaleFactorManager.__init__, after self._topics = TopicManager(self):
  from std_msgs.msg import Float64
  self._scale_factor_pub = self.create_publisher(
      Float64, "/tello_autonomy/scale_factor", 10)

  # In _poll_results(), right after:
  #   self.current_scale_factors[map_id] = result["scale_factor"]
  # add:
  msg = Float64()
  msg.data = result["scale_factor"]
  self._scale_factor_pub.publish(msg)

Without this patch the script still works fine -- it just reports a
correction ratio (e.g. "x1.35") instead of an absolute number.
"""

import math
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

# Allow running this file from anywhere inside the tello_autonomy tree.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_TELLO_AUTONOMY_ROOT = os.path.dirname(_THIS_DIR)
if _TELLO_AUTONOMY_ROOT not in sys.path:
    sys.path.insert(0, _TELLO_AUTONOMY_ROOT)

from config import constants  # noqa: E402


SCALE_FACTOR_TOPIC = "/tello_autonomy/scale_factor"


class ScaleCalibrator(Node):
    def __init__(self):
        super().__init__("scale_calibrator")

        self._latest_pose = None
        self._pose_ready_event = threading.Event()
        self._latest_scale_factor = None  # only populated if the optional patch is applied

        self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_METRIC,
            self._on_pose,
            10,
        )
        # Harmless to subscribe even if nothing publishes this yet --
        # you'll just never get a value, and the distance math below
        # doesn't depend on it.
        self.create_subscription(
            Float64,
            SCALE_FACTOR_TOPIC,
            self._on_scale_factor,
            10,
        )

    def _on_pose(self, msg: PoseStamped):
        self._latest_pose = msg.pose.position
        if not self._pose_ready_event.is_set():
            self._pose_ready_event.set()

    def _on_scale_factor(self, msg: Float64):
        self._latest_scale_factor = msg.data

    def get_position(self):
        p = self._latest_pose
        if p is None:
            return None
        return (p.x, p.y, p.z)


def spin_in_background(node):
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return thread


def wait_for_scale_ready(node, timeout_sec=120.0):
    print("Waiting for scale factor to become available...")
    print(
        f"(this happens once {constants.MIN_KEYFRAMES_FOR_SCALE_FACTOR}+ keyframes "
        f"exist and the first Depth-Anything-V2 pass completes -- can take "
        f"~30-90s after takeoff/handshake)"
    )
    ready = node._pose_ready_event.wait(timeout=timeout_sec)
    if not ready:
        print(
            f"\nTimed out after {timeout_sec:.0f}s waiting for scale factor. "
            f"Check that main.py is running, SLAM has ACKed, and enough "
            f"keyframes have accumulated."
        )
        return False
    print("\n*** Scale factor is ready -- current_pose_metric is now live. ***")
    if node._latest_scale_factor is not None:
        print(f"Reported scale factor: {node._latest_scale_factor:.6f}")
    else:
        print(
            "(No numeric scale factor available -- the optional patch in this "
            "file's docstring isn't applied. You'll still get a correction "
            "ratio at the end.)"
        )
    return True


def main():
    rclpy.init()
    node = ScaleCalibrator()
    spin_in_background(node)

    if not wait_for_scale_ready(node):
        rclpy.shutdown()
        return

    print("\n" + "=" * 60)
    print(" SCALE FACTOR CALIBRATION")
    print("=" * 60)
    print("Hold the drone steady at your START position.")
    input("Press ENTER when ready to mark the start point...")

    start = node.get_position()
    if start is None:
        print("No pose available -- aborting.")
        rclpy.shutdown()
        return
    print(f"Start position recorded: x={start[0]:.3f}  y={start[1]:.3f}  z={start[2]:.3f}")

    print(
        "\nNow move the drone a KNOWN real-world distance (tape measure "
        "recommended -- a straight-line 2m push works well)."
    )
    print("Live displacement from start will print below as you move.\n")

    stop_event = threading.Event()

    def live_display():
        while not stop_event.is_set():
            pos = node.get_position()
            if pos is not None:
                dx = pos[0] - start[0]
                dy = pos[1] - start[1]
                dz = pos[2] - start[2]
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                print(
                    f"\r  live displacement: {dist:6.3f} m   "
                    f"(dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f})   ",
                    end="",
                    flush=True,
                )
            stop_event.wait(0.1)

    display_thread = threading.Thread(target=live_display, daemon=True)
    display_thread.start()

    input(
        "\n\nPress ENTER when you have finished the move and the drone is "
        "steady at the END position..."
    )
    stop_event.set()
    display_thread.join(timeout=1.0)

    end = node.get_position()
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    measured_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

    print(f"\n\nEnd position recorded:   x={end[0]:.3f}  y={end[1]:.3f}  z={end[2]:.3f}")
    print(
        f"Measured displacement:   {measured_dist:.3f} m   "
        f"(dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f})"
    )

    while True:
        try:
            real_dist_str = input(
                "\nEnter the REAL distance you actually moved the drone "
                "(meters, e.g. 2.0): "
            ).strip()
            real_dist = float(real_dist_str)
            if real_dist <= 0:
                print("Must be a positive number.")
                continue
            break
        except ValueError:
            print("Not a valid number, try again.")

    error_abs = measured_dist - real_dist
    error_pct = (error_abs / real_dist) * 100.0
    correction_ratio = real_dist / measured_dist if measured_dist > 1e-6 else float("nan")

    print("\n" + "=" * 60)
    print(" CALIBRATION RESULT")
    print("=" * 60)
    print(f"  Real distance moved:      {real_dist:.3f} m")
    print(f"  SLAM-reported distance:   {measured_dist:.3f} m")
    print(f"  Absolute error:           {error_abs:+.3f} m")
    print(f"  Percent error:            {error_pct:+.1f} %")

    if node._latest_scale_factor is not None:
        suggested = node._latest_scale_factor * correction_ratio
        print(f"\n  Current scale factor:     {node._latest_scale_factor:.6f}")
        print(f"  Suggested corrected:      {suggested:.6f}  (= current * real/measured)")
    else:
        print(
            f"\n  Correction ratio needed:  x{correction_ratio:.4f}  "
            f"(current_scale * {correction_ratio:.4f} = suggested_scale)"
        )
        print(
            "  (No live scale_factor topic seen -- apply the optional patch "
            "described at the top of this file for the exact numeric value.)"
        )

    if abs(error_pct) < 5.0:
        print("\n  -> Within 5% error: scale factor looks reasonably accurate.")
    elif abs(error_pct) < 15.0:
        print("\n  -> 5-15% error: usable but worth tightening.")
    else:
        print(
            "\n  -> >15% error: scale factor is significantly off. Don't trust "
            "planner bounds/waypoint distances until this improves."
        )

    print("=" * 60)
    print("\nTip: run this a few times (different distances/directions) before")
    print("drawing conclusions -- a single sample can be noisy.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
