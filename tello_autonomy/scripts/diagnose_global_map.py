#!/usr/bin/env python3
"""
scripts/diagnose_global_map.py

AI & Human Diagnostic Tool for Autonomous SLAM Map Alignment.
Reads 'global_map_diagnostics.json' and 'global_keyframe_pixels.csv'
to detect scale discontinuities, alignment jump vectors, frame drifts, and raycasting errors.
"""

import json
import os
import numpy as np

DIAGNOSTICS_JSON = "/home/hamdan/autonomous_drone/global_map_diagnostics.json"
KEYFRAME_CSV = "/home/hamdan/autonomous_drone/global_keyframe_pixels.csv"


def run_diagnostics():
    print("=" * 70)
    print("      AUTONOMOUS SLAM GLOBAL MAP ALIGNMENT DIAGNOSTIC REPORT")
    print("=" * 70)

    if not os.path.exists(DIAGNOSTICS_JSON):
        print(f"[ERROR] Diagnostics file not found: {DIAGNOSTICS_JSON}")
        print("Run the Tello autonomy stack first to generate live flight logs.")
        return

    with open(DIAGNOSTICS_JSON, "r") as f:
        diag = json.load(f)

    summary = diag.get("summary", {})
    active_maps = diag.get("active_maps", {})
    events = diag.get("map_alignment_events", [])
    samples = diag.get("trajectory_samples", [])
    per_map_summ = diag.get("per_map_trajectory_summary", {})

    print(f"\n--- SESSION SUMMARY ---")
    print(f"Start Time: {diag.get('session_start_iso')}")
    print(f"Total Maps Detected: {len(active_maps)} (Max map_id: {summary.get('max_map_id')})")
    print(f"Total Alignment Transitions: {summary.get('total_transitions')}")
    print(f"Accepted Alignments: {summary.get('accepted_alignments')}")
    print(f"Rejected (Hard Reset) Alignments: {summary.get('rejected_alignments')}")

    print(f"\n--- ACTIVE SUBMAPS & LIFETIMES ---")
    for map_id_str, info in sorted(active_maps.items(), key=lambda x: int(x[0])):
        print(f"  Map ID {map_id_str}: First seen {info['first_seen_sec']}s | "
              f"Last seen {info['last_seen_sec']}s | Poses recorded: {info['pose_count']}")

    print(f"\n--- MAP ALIGNMENT TRANSITION DETAILS ---")
    if not events:
        print("  No map alignment transitions recorded yet.")
    else:
        for idx, ev in enumerate(events, 1):
            status = "ACCEPTED" if ev['accepted'] else "REJECTED (Hard Reset)"
            t = ev['translation_global']
            t_mag = np.linalg.norm(t)
            print(f"  [{idx}] Map {ev['old_map_id']} -> Map {ev['new_map_id']} | {status}")
            print(f"      Incremental offset: {ev['offset_m']} m | Gap: {ev['gap_sec']} s")
            print(f"      Cumulative global T: {t}  (|T|={t_mag:.3f}m)")
            print(f"      Rotation Quat (x,y,z,w): {ev['rotation_quat']}")

    # --- Per-map spatial summary (never discarded, covers ALL maps) ---
    print(f"\n--- PER-MAP SPATIAL EXTENTS (all maps, never discarded) ---")
    if not per_map_summ:
        print("  No per-map summary yet (need at least one flight).")
    else:
        for mid_str, info in sorted(per_map_summ.items(), key=lambda x: int(x[0])):
            fp = info['first_global_metric_pose']
            lp = info['last_global_metric_pose']
            mn = info['min_xyz']
            mx = info['max_xyz']
            span = np.linalg.norm(np.array(mx) - np.array(mn))
            print(f"  Map {mid_str}: {info['sample_count']} samples | "
                  f"t={info['first_pose_t']:.1f}s → {info['last_pose_t']:.1f}s")
            print(f"    First global pose: {fp}")
            print(f"    Last  global pose: {lp}")
            print(f"    X: [{mn[0]:.3f}, {mx[0]:.3f}]  "
                  f"Y: [{mn[1]:.3f}, {mx[1]:.3f}]  "
                  f"Z: [{mn[2]:.3f}, {mx[2]:.3f}]  span={span:.3f}m")

    # --- Alignment cross-check: do map extents overlap sensibly? ---
    print(f"\n--- ALIGNMENT CROSS-CHECK (first pose of new map vs last pose of old map) ---")
    for ev in events:
        old_str = str(ev['old_map_id'])
        new_str = str(ev['new_map_id'])
        if old_str in per_map_summ and new_str in per_map_summ:
            old_last = np.array(per_map_summ[old_str]['last_global_metric_pose'])
            new_first = np.array(per_map_summ[new_str]['first_global_metric_pose'])
            gap_m = np.linalg.norm(new_first - old_last)
            status = "OK" if gap_m < 1.5 else "WARNING - possible jump"
            print(f"  Map {ev['old_map_id']} last  → Map {ev['new_map_id']} first: {gap_m:.3f}m gap [{status}]")
            print(f"    old last : {old_last.tolist()}")
            print(f"    new first: {new_first.tolist()}")
        else:
            print(f"  Map {ev['old_map_id']} -> {ev['new_map_id']}: summary not available yet.")

    # --- Revolving detail buffer: check for intra-session jumps ---
    print(f"\n--- TRAJECTORY DETAIL BUFFER: CONSECUTIVE STEP CHECK ---")
    if len(samples) > 1:
        prev = samples[0]
        max_jump = 0.0
        jump_event = None
        for s in samples[1:]:
            p1 = np.array(prev["global_metric_pose"])
            p2 = np.array(s["global_metric_pose"])
            dist = np.linalg.norm(p2 - p1)
            if dist > max_jump:
                max_jump = dist
                jump_event = (prev, s)
            prev = s

        print(f"  Detail buffer samples: {len(samples)}")
        print(f"  Max consecutive step: {max_jump:.3f}m")
        if max_jump > 1.0 and jump_event:
            a, b = jump_event
            print(f"  [WARNING] Jump detected: Map {a['map_id']} t={a['t_rel_sec']}s → "
                  f"Map {b['map_id']} t={b['t_rel_sec']}s ({max_jump:.3f}m)")
            print(f"    before: {a['global_metric_pose']}")
            print(f"    after : {b['global_metric_pose']}")
        else:
            print("  [PASS] No unexpected spatial jumps in detail buffer.")
    else:
        print("  Insufficient samples in detail buffer yet.")

    print(f"\n--- KEYFRAME LANDMARK PIXEL AUDIT ---")
    if os.path.exists(KEYFRAME_CSV):
        try:
            import csv
            map_stats = {}
            with open(KEYFRAME_CSV) as cf:
                reader = csv.DictReader(cf)
                for row in reader:
                    mid = int(row['map_id'])
                    if mid not in map_stats:
                        map_stats[mid] = {'u': [], 'v': [], 'd': []}
                    map_stats[mid]['u'].append(float(row['pixel_u']))
                    map_stats[mid]['v'].append(float(row['pixel_v']))
                    map_stats[mid]['d'].append(float(row['depth_camera_frame']))

            total = sum(len(v['u']) for v in map_stats.values())
            print(f"  Loaded Keyframe CSV: {total} landmark observations across {len(map_stats)} maps.")
            for mid in sorted(map_stats.keys()):
                us = map_stats[mid]['u']
                vs = map_stats[mid]['v']
                ds = map_stats[mid]['d']
                print(f"    Map {mid}: {len(us)} points | "
                      f"Mean pixel (u,v)=({np.mean(us):.1f},{np.mean(vs):.1f}) | "
                      f"Mean depth={np.mean(ds):.3f}m | "
                      f"Depth range=[{min(ds):.3f}, {max(ds):.3f}]m")
        except Exception as e:
            print(f"  [ERROR] Could not parse keyframe CSV: {e}")
    else:
        print(f"  Keyframe CSV not found: {KEYFRAME_CSV}")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    run_diagnostics()
