#!/usr/bin/env python3
"""
scripts/verify_occupancy_map.py

Mathematical & Diagnostic verification node for Occupancy Map:
1. Topic frequency & latency tracking for Pose and PointCloud2.
2. Pose-PointCloud timestamp synchronization delta (sync slop).
3. Metric PointCloud density & bounding box (X_min/max, Y_min/max, Z_min/max).
4. OctoMap binary payload size, update count, and message stats.
5. Prints periodic live mathematical report to terminal.
"""

import os
import sys
import time
import math
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from std_msgs.msg import Float64

_TELLO_AUTONOMY_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TELLO_AUTONOMY_ROOT not in sys.path:
    sys.path.insert(0, _TELLO_AUTONOMY_ROOT)

from config import constants


class OccupancyMapVerifier(Node):
    def __init__(self):
        super().__init__('occupancy_map_verifier')

        # --- Subscriptions ---
        self.create_subscription(
            PoseStamped,
            constants.TOPIC_CURRENT_POSE_METRIC,
            self._pose_callback,
            10
        )
        self.create_subscription(
            PointCloud2,
            constants.TOPIC_CURRENT_POINTS_METRIC,
            self._points_callback,
            10
        )
        self.create_subscription(
            Octomap,
            constants.TOPIC_OCCUPANCY_GRID,
            self._octomap_callback,
            10
        )
        self.create_subscription(
            Float64,
            "/tello_autonomy/scale_factor",
            self._scale_callback,
            10
        )

        # --- Diagnostics State ---
        self._last_pose_msg = None
        self._last_pose_time = 0.0
        self._pose_count = 0

        self._last_points_msg = None
        self._last_points_time = 0.0
        self._points_count = 0

        self._last_octomap_msg = None
        self._octomap_count = 0

        self._current_scale_factor = 1.0

        # Stats history
        self._start_time = time.time()
        self._sync_slops = []

        # Periodic Diagnostic Timer (prints report every 2.0 seconds)
        self._timer = self.create_wall_timer(2.0, self._print_report)

        self.get_logger().info("Occupancy Map Verifier started. Listening to metric pose, points, and octomap topics...")

    def _pose_callback(self, msg: PoseStamped):
        self._last_pose_msg = msg
        self._last_pose_time = time.time()
        self._pose_count += 1
        self._check_sync()

    def _points_callback(self, msg: PointCloud2):
        self._last_points_msg = msg
        self._last_points_time = time.time()
        self._points_count += 1
        self._check_sync()

    def _octomap_callback(self, msg: Octomap):
        self._last_octomap_msg = msg
        self._octomap_count += 1

    def _scale_callback(self, msg: Float64):
        self._current_scale_factor = msg.data

    def _check_sync(self):
        if self._last_pose_msg is not None and self._last_points_msg is not None:
            t_pose = self._last_pose_msg.header.stamp.sec + self._last_pose_msg.header.stamp.nanosec * 1e-9
            t_pts = self._last_points_msg.header.stamp.sec + self._last_points_msg.header.stamp.nanosec * 1e-9
            dt = abs(t_pose - t_pts)
            self._sync_slops.append(dt)
            if len(self._sync_slops) > 50:
                self._sync_slops.pop(0)

    def _parse_cloud_bounds(self, msg: PointCloud2):
        """Extract point count and min/max bounding box from PointCloud2."""
        if msg is None or msg.width * msg.height == 0:
            return 0, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        num_points = msg.width * msg.height
        # Find offset of x, y, z fields
        x_off, y_off, z_off = None, None, None
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset

        if x_off is None or y_off is None or z_off is None:
            return num_points, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        point_step = msg.point_step
        data = msg.data

        x_min, x_max = float('inf'), float('-inf')
        y_min, y_max = float('inf'), float('-inf')
        z_min, z_max = float('inf'), float('-inf')

        # Sample up to 500 points for speed
        stride = max(1, num_points // 500)
        valid_points = 0
        for i in range(0, num_points, stride):
            offset = i * point_step
            if offset + 12 <= len(data):
                x, y, z = struct.unpack_from('fff', data, offset)
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                    valid_points += 1
                    x_min = min(x_min, x)
                    x_max = max(x_max, x)
                    y_min = min(y_min, y)
                    y_max = max(y_max, y)
                    z_min = min(z_min, z)
                    z_max = max(z_max, z)

        if valid_points == 0:
            return num_points, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        return num_points, (x_min, x_max, y_min, y_max, z_min, z_max)

    def _print_report(self):
        elapsed = max(0.1, time.time() - self._start_time)
        pose_hz = self._pose_count / elapsed
        pts_hz = self._points_count / elapsed
        map_hz = self._octomap_count / elapsed

        avg_slop_ms = (sum(self._sync_slops) / len(self._sync_slops) * 1000.0) if self._sync_slops else 0.0

        pt_count, (xmin, xmax, ymin, ymax, zmin, zmax) = self._parse_cloud_bounds(self._last_points_msg)
        dx, dy, dz = xmax - xmin, ymax - ymin, zmax - zmin

        pose_str = "N/A"
        if self._last_pose_msg:
            p = self._last_pose_msg.pose.position
            pose_str = f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m"

        octo_str = "N/A"
        if self._last_octomap_msg:
            octo_str = f"res={self._last_octomap_msg.resolution:.2f}m, binary_bytes={len(self._last_octomap_msg.data)}"

        report = (
            "\n"
            "===========================================================\n"
            "          OCCUPANCY MAP & METRIC AUDIT REPORT              \n"
            "===========================================================\n"
            f" Runtime Elapsed:       {elapsed:.1f} s\n"
            f" Scale Factor:          {self._current_scale_factor:.4f}\n"
            "-----------------------------------------------------------\n"
            " Topic Frequencies:\n"
            f"   - Metric Pose:       {pose_hz:.1f} Hz (Total: {self._pose_count})\n"
            f"   - Metric PointCloud: {pts_hz:.1f} Hz (Total: {self._points_count})\n"
            f"   - OctoMap Updates:   {map_hz:.1f} Hz (Total: {self._octomap_count})\n"
            f" Pose-Cloud Sync Slop:  {avg_slop_ms:.2f} ms (Target < 50ms)\n"
            "-----------------------------------------------------------\n"
            " Current Metric Pose:   " + pose_str + "\n"
            f" PointCloud Density:    {pt_count} points/frame\n"
            " PointCloud Bounding Box (Metric Dimensions):\n"
            f"   - X Range: [{xmin:.2f}, {xmax:.2f}] m  (Span: {dx:.2f} m)\n"
            f"   - Y Range: [{ymin:.2f}, {ymax:.2f}] m  (Span: {dy:.2f} m)\n"
            f"   - Z Range: [{zmin:.2f}, {zmax:.2f}] m  (Span: {dz:.2f} m)\n"
            "-----------------------------------------------------------\n"
            " OctoMap Grid Info:     " + octo_str + "\n"
            "===========================================================\n"
        )

        print(report)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
