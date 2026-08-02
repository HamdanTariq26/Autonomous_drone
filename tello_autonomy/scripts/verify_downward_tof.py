#!/usr/bin/env python3
"""
scripts/verify_downward_tof.py

Standalone sanity-check script for the Tello/RMTT's DOWNWARD ToF sensor.
No ROS2, no takeoff required - connect to the drone's WiFi, run this, and
hold/move the drone at different heights over the floor to watch the
readings change live.

Reads directly from the Tello state stream (UDP 8890), which already
carries the downward ToF as the 'tof' field (mm precision, reported in cm
by djitellopy), plus barometer height and battery as a cross-check.

Usage:
    pip install djitellopy
    # Connect your PC's WiFi to the Tello's hotspot first
    python3 scripts/verify_downward_tof.py
"""

import time
from djitellopy import Tello

TOF_OUT_OF_RANGE_CM = 8192  # sentinel value the SDK returns when nothing is detected
POLL_HZ = 5


def main():
    tello = Tello()

    print("Connecting to Tello...")
    tello.connect()
    print(f"Connected. Battery: {tello.get_battery()}%")

    print("\nHold the drone at different heights above the floor (or a hand/box)")
    print("and watch the ToF distance change. Press Ctrl+C to stop.\n")
    print(f"{'ToF (cm)':>10} | {'Height (cm)':>11} | {'Baro (m)':>9} | {'Battery':>7} | Status")
    print("-" * 62)

    period = 1.0 / POLL_HZ
    try:
        while True:
            state = tello.get_current_state()

            tof_cm = state.get("tof")
            height_cm = state.get("h")
            baro_m = state.get("baro")
            battery = state.get("bat")

            if tof_cm is None:
                status = "NO STATE DATA YET - is the drone powered on and connected?"
            elif tof_cm >= TOF_OUT_OF_RANGE_CM:
                status = "OUT OF RANGE (nothing detected below)"
            elif tof_cm <= 0:
                status = "SUSPICIOUS (<=0, check sensor)"
            else:
                status = "OK"

            print(
                f"{str(tof_cm):>10} | {str(height_cm):>11} | "
                f"{str(baro_m):>9} | {str(battery):>7} | {status}"
            )

            time.sleep(period)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        try:
            tello.end()
        except Exception:
            pass
        print("Disconnected cleanly.")


if __name__ == "__main__":
    main()
