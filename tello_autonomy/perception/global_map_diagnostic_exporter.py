"""
perception/global_map_diagnostic_exporter.py

Diagnostic logger node for Autonomous SLAM Map Alignment.
Continuously records map IDs, raw poses, metric poses, scale factors, alignment
matrices, and keyframe landmark statistics into an AI-parseable JSON file:
'global_map_diagnostics.json'.

Trajectory storage strategy (avoids the revolving-buffer overwrite problem):
  1. per_map_trajectory_summary  — always kept, one dict per map_id with:
       first_pose, last_pose, min/max extents, total_sample_count
     This is NEVER discarded regardless of flight duration.
  2. trajectory_samples — a downsampled revolving buffer (every SAMPLE_STRIDE-th
     message, max BUFFER_CAP records).  Provides detail for the most recent
     portion of flight for jump-detection analysis.
"""

import json
import os
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tello_autonomy_msgs.msg import MapAlignment
from perception.live_scaler import parse_map_id_from_frame_id
from config import constants

# Keep one detail sample per this many metric pose callbacks per map_id.
# At ~30 Hz metric pose, stride=15 → ~2 samples/s → ~120 samples/min per map.
SAMPLE_STRIDE = 15
# Hard cap on the revolving detail buffer (across all maps).
BUFFER_CAP = 2000


class GlobalMapDiagnosticExporter(Node):
    """
    Subscribes to SLAM telemetry and map alignment events, exporting a rich,
    structured JSON file detailing global map registration, scale factors, and
    spatial trajectories for AI or offline debugging.
    """

    def __init__(self, output_path="/home/hamdan/autonomous_drone/global_map_diagnostics.json", node_name="global_map_diagnostic_exporter"):
        super().__init__(node_name)
        self._output_path = output_path
        self._start_time = time.time()

        # Telemetry storage
        self._diagnostics_data = {
            "session_start_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(self._start_time)),
            "active_maps": {},                # map_id -> {first_seen_sec, last_seen_sec, pose_count}
            "map_alignment_events": [],       # alignment transition dicts (always kept)
            "per_map_trajectory_summary": {}, # map_id -> {first_pose, last_pose, min_xyz, max_xyz, count}
            "trajectory_samples": [],         # downsampled revolving detail buffer
            "summary": {
                "total_transitions": 0,
                "accepted_alignments": 0,
                "rejected_alignments": 0,
                "max_map_id": 0,
            }
        }

        # Per-map callback counter for stride-based downsampling
        self._metric_cb_count = {}  # map_id -> int

        # ROS2 Subscriptions
        self._pose_raw_sub = self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_RAW,
            self._on_pose_raw,
            10
        )
        self._pose_metric_sub = self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_METRIC,
            self._on_pose_metric,
            10
        )
        self._alignment_sub = self.create_subscription(
            MapAlignment,
            constants.TOPIC_MAP_ALIGNMENT,
            self._on_map_alignment,
            10
        )

        # Periodic dump timer (every 2.0 seconds)
        self._dump_timer = self.create_timer(2.0, self._export_to_file)
        self._last_raw_pose = None
        self.get_logger().info(f"GlobalMapDiagnosticExporter active. Writing to: {self._output_path}")

    # ------------------------------------------------------------------
    def _on_pose_raw(self, msg: PoseStamped):
        map_id = parse_map_id_from_frame_id(msg.header.frame_id)
        if map_id is None:
            return

        now_sec = time.time() - self._start_time
        if map_id not in self._diagnostics_data["active_maps"]:
            self._diagnostics_data["active_maps"][map_id] = {
                "first_seen_sec": round(now_sec, 3),
                "last_seen_sec": round(now_sec, 3),
                "pose_count": 1
            }
        else:
            self._diagnostics_data["active_maps"][map_id]["last_seen_sec"] = round(now_sec, 3)
            self._diagnostics_data["active_maps"][map_id]["pose_count"] += 1

        self._diagnostics_data["summary"]["max_map_id"] = max(
            self._diagnostics_data["summary"]["max_map_id"], map_id
        )
        self._last_raw_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, map_id)

    # ------------------------------------------------------------------
    def _on_pose_metric(self, msg: PoseStamped):
        if self._last_raw_pose is None:
            return

        map_id = parse_map_id_from_frame_id(msg.header.frame_id)
        raw_x, raw_y, raw_z, raw_map_id = self._last_raw_pose
        if raw_map_id != map_id:
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gz = msg.pose.position.z
        now_sec = round(time.time() - self._start_time, 3)

        # --- 1. Always update per-map trajectory summary (never discarded) ---
        key = str(map_id)
        summ = self._diagnostics_data["per_map_trajectory_summary"]
        if key not in summ:
            summ[key] = {
                "map_id": map_id,
                "first_pose_t": now_sec,
                "first_global_metric_pose": [round(gx, 4), round(gy, 4), round(gz, 4)],
                "first_raw_pose": [round(raw_x, 4), round(raw_y, 4), round(raw_z, 4)],
                "last_pose_t": now_sec,
                "last_global_metric_pose": [round(gx, 4), round(gy, 4), round(gz, 4)],
                "last_raw_pose": [round(raw_x, 4), round(raw_y, 4), round(raw_z, 4)],
                "min_xyz": [round(gx, 4), round(gy, 4), round(gz, 4)],
                "max_xyz": [round(gx, 4), round(gy, 4), round(gz, 4)],
                "sample_count": 1,
            }
        else:
            entry = summ[key]
            entry["last_pose_t"] = now_sec
            entry["last_global_metric_pose"] = [round(gx, 4), round(gy, 4), round(gz, 4)]
            entry["last_raw_pose"] = [round(raw_x, 4), round(raw_y, 4), round(raw_z, 4)]
            entry["min_xyz"] = [
                round(min(entry["min_xyz"][0], gx), 4),
                round(min(entry["min_xyz"][1], gy), 4),
                round(min(entry["min_xyz"][2], gz), 4),
            ]
            entry["max_xyz"] = [
                round(max(entry["max_xyz"][0], gx), 4),
                round(max(entry["max_xyz"][1], gy), 4),
                round(max(entry["max_xyz"][2], gz), 4),
            ]
            entry["sample_count"] += 1

        # --- 2. Stride-based downsampling into revolving detail buffer ---
        cnt = self._metric_cb_count.get(map_id, 0) + 1
        self._metric_cb_count[map_id] = cnt
        if cnt % SAMPLE_STRIDE != 0:
            return  # skip this frame; only record every SAMPLE_STRIDE-th

        sample = {
            "t_rel_sec": now_sec,
            "map_id": map_id,
            "raw_pose": [round(raw_x, 4), round(raw_y, 4), round(raw_z, 4)],
            "global_metric_pose": [round(gx, 4), round(gy, 4), round(gz, 4)],
            "orientation_quat": [
                round(msg.pose.orientation.x, 4),
                round(msg.pose.orientation.y, 4),
                round(msg.pose.orientation.z, 4),
                round(msg.pose.orientation.w, 4)
            ]
        }
        buf = self._diagnostics_data["trajectory_samples"]
        if len(buf) >= BUFFER_CAP:
            buf.pop(0)
        buf.append(sample)

    # ------------------------------------------------------------------
    def _on_map_alignment(self, msg: MapAlignment):
        now_sec = round(time.time() - self._start_time, 3)
        event = {
            "timestamp_sec": now_sec,
            "old_map_id": int(msg.old_map_id),
            "new_map_id": int(msg.new_map_id),
            "accepted": bool(msg.accepted),
            "offset_m": round(float(msg.offset_m), 4),
            "gap_sec": round(float(msg.gap_sec), 3),
            "translation_global": [
                round(float(msg.translation.x), 4),
                round(float(msg.translation.y), 4),
                round(float(msg.translation.z), 4)
            ],
            "rotation_quat": [
                round(float(msg.rotation.x), 4),
                round(float(msg.rotation.y), 4),
                round(float(msg.rotation.z), 4),
                round(float(msg.rotation.w), 4)
            ]
        }

        self._diagnostics_data["map_alignment_events"].append(event)
        self._diagnostics_data["summary"]["total_transitions"] += 1
        if msg.accepted:
            self._diagnostics_data["summary"]["accepted_alignments"] += 1
        else:
            self._diagnostics_data["summary"]["rejected_alignments"] += 1

        self.get_logger().info(f"[DIAGNOSTICS] Alignment logged for map {msg.old_map_id} -> {msg.new_map_id} (accepted={msg.accepted})")

    # ------------------------------------------------------------------
    def _export_to_file(self):
        try:
            tmp_path = self._output_path + ".tmp"
            with open(tmp_path, "w") as f:
                json.dump(self._diagnostics_data, f, indent=2)
            os.replace(tmp_path, self._output_path)
        except Exception as e:
            self.get_logger().error(f"Failed to export diagnostics JSON: {e}")
