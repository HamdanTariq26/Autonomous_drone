#!/usr/bin/env python3
"""
scripts/simulated_drone_maze.py

Real-World Multi-Room Building Environment Simulator for ROS2.
- Exact Ray-AABB surface intersection math (0.00 mm wall penetration).
- 4 Interconnected Rooms: Main Reception, Conference Hall, Open Workstations, Executive Corridor.
- 0% hardcoded paths: Drone flight is driven 100% dynamically by the C++ exploration_cpp planner.
"""

import math
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tello_autonomy_msgs.srv import NbvPlan

from config import constants


def ray_box_intersection(origin, direction, box):
    """
    Computes exact Ray-AABB (Axis-Aligned Bounding Box) surface intersection.
    Returns exact parametric distance 't' to the outer wall skin, or None if no hit.
    """
    min_x, max_x, min_y, max_y, min_z, max_z = box
    ox, oy, oz = origin
    dx, dy, dz = direction

    t_min = 0.0
    t_max = 1e6

    # X axis
    if abs(dx) < 1e-9:
        if ox < min_x or ox > max_x:
            return None
    else:
        inv_d = 1.0 / dx
        t1 = (min_x - ox) * inv_d
        t2 = (max_x - ox) * inv_d
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)

    # Y axis
    if abs(dy) < 1e-9:
        if oy < min_y or oy > max_y:
            return None
    else:
        inv_d = 1.0 / dy
        t1 = (min_y - oy) * inv_d
        t2 = (max_y - oy) * inv_d
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)

    # Z axis
    if abs(dz) < 1e-9:
        if oz < min_z or oz > max_z:
            return None
    else:
        inv_d = 1.0 / dz
        t1 = (min_z - oz) * inv_d
        t2 = (max_z - oz) * inv_d
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)

    if t_max >= t_min and t_min > 0.0:
        return t_min
    return None


class SimulatedDroneMaze(Node):
    def __init__(self):
        super().__init__("simulated_drone_maze")

        # --- Publishers ---
        self._pose_pub = self.create_publisher(
            PoseStamped, constants.TOPIC_CURRENT_POSE_METRIC, 10
        )
        self._pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, "pose", 10
        )
        self._points_pub = self.create_publisher(
            PointCloud2, constants.TOPIC_CURRENT_POINTS_METRIC, 10
        )
        self._maze_viz_pub = self.create_publisher(
            MarkerArray, "/simulated_maze_visualization", 10
        )
        self._drone_bbx_pub = self.create_publisher(
            Marker, "/drone_bounding_box", 10
        )

        # --- Service Client for exploration_cpp ---
        self._nbv_client = self.create_client(NbvPlan, "nbvplanner")

        # --- Drone State (x, y, z, yaw) ---
        self.drone_x = -2.0  # Start inside Room 1 (Main Entrance)
        self.drone_y = -2.0
        self.drone_z = 0.8   # Flight altitude 0.8m
        self.drone_yaw = 0.0

        # --- Dynamic Active Path Queue ---
        self.active_path_waypoints = []
        self.planning_in_progress = False

        # --- REAL-WORLD 4-ROOM ARCHITECTURAL FLOORPLAN OBSTACLES ---
        # 10m x 10m x 2.8m Real-World Building Layout
        self.obstacles = [
            # Outer Building Boundary Walls
            (-5.0, 5.0,  4.9, 5.1,  0.0, 2.8),  # North Wall
            (-5.0, 5.0, -5.1, -4.9, 0.0, 2.8),  # South Wall
            ( 4.9, 5.1, -5.0, 5.0,  0.0, 2.8),  # East Wall
            (-5.1, -4.9, -5.0, 5.0,  0.0, 2.8),  # West Wall

            # Central Dividing Wall X = 0.0 (Doorway gap at y = -1.0 .. 1.0)
            (-0.1, 0.1, -4.9, -1.0, 0.0, 2.8),  # South Wall Section
            (-0.1, 0.1,  1.0,  4.9, 0.0, 2.8),  # North Wall Section

            # Central Dividing Wall Y = 0.0 (Doorway gaps at x = -3.0..-2.0 and x = 2.0..3.0)
            (-4.9, -3.0, -0.1, 0.1, 0.0, 2.8),  # West Partition
            (-2.0,  2.0, -0.1, 0.1, 0.0, 2.8),  # Middle Partition
            ( 3.0,  4.9, -0.1, 0.1, 0.0, 2.8),  # East Partition

            # Room 2: Conference Room Furniture (x = 2.0, y = -2.0)
            (1.2, 2.8, -2.6, -1.4, 0.0, 0.75),  # Conference Table
            (1.5, 2.5, -3.8, -3.4, 0.0, 1.6),   # Whiteboard / TV Stand

            # Room 3: Open Office Workstations (x = 2.0, y = 2.0)
            (1.0, 2.2,  1.5, 2.5, 0.0, 0.75),   # Desk 1
            (2.8, 3.8,  1.5, 2.5, 0.0, 0.75),   # Desk 2
            (3.4, 3.8,  3.0, 3.8, 0.0, 1.8),   # Storage Cabinet

            # Room 4: Executive Suite (x = -2.0, y = 2.0)
            (-3.5, -2.5,  1.8, 2.8, 0.0, 0.75),  # Executive Desk
            (-3.8, -3.2,  3.2, 3.8, 0.0, 2.0),   # Bookshelf

            # Room 1: Reception Pillars & Furniture (x = -2.0, y = -2.0)
            (-3.5, -2.8, -1.2, -0.8, 0.0, 0.50), # Coffee Table
            (-1.0, -0.6, -3.5, -3.1, 0.0, 2.8),  # Structural Support Column
        ]

        # Camera FOV parameters (Matching Tello: 82° H, 60° V, range 3.5m)
        self.fov_h = math.radians(82.0)
        self.fov_v = math.radians(60.0)
        self.max_range = 3.5
        self.ticks = 0

        # Timers
        self.sim_timer = self.create_timer(0.05, self._sim_step)  # 20 Hz simulation tick
        self.viz_timer = self.create_timer(1.0, self._publish_maze_visualization)  # 1 Hz RViz maze markers

        self.get_logger().info("Real-World Building Simulator Started! (Exact Surface Ray-AABB Math Active)")

    def _publish_maze_visualization(self):
        """Publishes 3D markers for the ground-truth real-world building walls in RViz2."""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for idx, obs in enumerate(self.obstacles):
            min_x, max_x, min_y, max_y, min_z, max_z = obs
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "map"
            marker.ns = "ground_truth_maze"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = (min_x + max_x) / 2.0
            marker.pose.position.y = (min_y + max_y) / 2.0
            marker.pose.position.z = (min_z + max_z) / 2.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = max(max_x - min_x, 0.05)
            marker.scale.y = max(max_y - min_y, 0.05)
            marker.scale.z = max(max_z - min_z, 0.05)

            marker.color.r = 0.1
            marker.color.g = 0.5
            marker.color.b = 0.9
            marker.color.a = 0.30

            marker_array.markers.append(marker)

        self._maze_viz_pub.publish(marker_array)

    def _publish_drone_bounding_box(self, stamp, frame_id, pose):
        """Publishes a 3D yellow box marker representing the drone's physical bounding box (0.20m x 0.20m x 0.14m)."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "drone_physical_bounding_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = pose

        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.14

        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.0
        marker.color.a = 0.55

        self._drone_bbx_pub.publish(marker)

    def _sim_step(self):
        """Executes flight steps along exploration_cpp waypoints and publishes camera pointclouds."""
        self.ticks += 1
        stamp = self.get_clock().now().to_msg()
        frame_id = "map"

        # 1. Publish Drone Pose (geometry_msgs/PoseStamped and PoseWithCovarianceStamped)
        cy = math.cos(self.drone_yaw * 0.5)
        sy = math.sin(self.drone_yaw * 0.5)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = self.drone_x
        pose_msg.pose.position.y = self.drone_y
        pose_msg.pose.position.z = self.drone_z
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = sy
        pose_msg.pose.orientation.w = cy
        self._pose_pub.publish(pose_msg)

        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header = pose_msg.header
        pose_cov_msg.pose.pose = pose_msg.pose
        self._pose_cov_pub.publish(pose_cov_msg)

        self._publish_drone_bounding_box(stamp, frame_id, pose_msg.pose)

        # 2. Ray-cast synthetic camera pointcloud with exact surface intersection math
        points = self._generate_synthetic_pointcloud()
        if points:
            header = Header()
            header.stamp = stamp
            header.frame_id = frame_id
            cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
            self._points_pub.publish(cloud_msg)

        # 3. Request new path from exploration_cpp after initial 3 seconds
        if self.ticks > 60 and not self.active_path_waypoints and not self.planning_in_progress:
            if self._nbv_client.service_is_ready():
                self.get_logger().info("Requesting Next-Best-View path from exploration_cpp (/nbvplanner)...")
                self.planning_in_progress = True
                req = NbvPlan.Request()
                req.header.stamp = stamp
                req.header.frame_id = "map"
                future = self._nbv_client.call_async(req)
                future.add_done_callback(self._on_nbv_path_received)

        # 4. Step drone along active_path_waypoints returned by exploration_cpp
        if self.active_path_waypoints:
            target_pose = self.active_path_waypoints[0]
            tx = target_pose.position.x
            ty = target_pose.position.y
            tz = target_pose.position.z

            qz = target_pose.orientation.z
            qw = target_pose.orientation.w
            target_yaw = 2.0 * math.atan2(qz, qw)

            dx = tx - self.drone_x
            dy = ty - self.drone_y
            dz = tz - self.drone_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist > 0.08:
                step = min(0.015, dist)  # Smooth 0.3 m/s motion
                next_x = self.drone_x + (dx / dist) * step
                next_y = self.drone_y + (dy / dist) * step
                next_z = self.drone_z + (dz / dist) * step

                # Check physical wall collision in simulator (safety margin 0.20m XY, drone half-height in Z)
                # FIX Bug2: old flat Z margin caused drone to falsely collide with low furniture it clears.
                # Use a proper 3-D AABB overlap test against the drone's physical bounding box instead.
                in_collision = False
                drone_half_h = 0.07   # half of 0.14 m drone body height
                drone_z_min = next_z - drone_half_h
                drone_z_max = next_z + drone_half_h
                for obs in self.obstacles:
                    min_x, max_x, min_y, max_y, min_z, max_z = obs
                    margin = 0.20
                    if (min_x - margin <= next_x <= max_x + margin and
                        min_y - margin <= next_y <= max_y + margin and
                        min_z <= drone_z_max and max_z >= drone_z_min):
                        in_collision = True
                        break

                if not in_collision:
                    self.drone_x = next_x
                    self.drone_y = next_y
                    self.drone_z = next_z
                    self.drone_yaw = target_yaw
                else:
                    # Physical obstacle encountered: stop step and discard rest of unmapped path
                    self.active_path_waypoints.clear()
            else:
                self.active_path_waypoints.pop(0)

    def _on_nbv_path_received(self, future):
        """Callback when exploration_cpp (/nbvplanner) returns a calculated RRT path."""
        self.planning_in_progress = False
        try:
            res = future.result()
            if res and res.path:
                self.get_logger().info(
                    f"Received dynamic RRT path with {len(res.path)} waypoints from exploration_cpp!"
                )
                self.active_path_waypoints = list(res.path)
            else:
                self.get_logger().warn("exploration_cpp returned empty path (retrying next tick)...")
        except Exception as e:
            self.get_logger().error(f"Service call to /nbvplanner failed: {e}")

    def _generate_synthetic_pointcloud(self):
        """Ray-casts from drone's camera pose using exact surface Ray-AABB intersection math."""
        points = []
        num_h = 40
        num_v = 30

        h_angles = np.linspace(-self.fov_h / 2.0, self.fov_h / 2.0, num_h)
        v_angles = np.linspace(-self.fov_v / 2.0, self.fov_v / 2.0, num_v)

        origin = (self.drone_x, self.drone_y, self.drone_z)
        cos_y = math.cos(self.drone_yaw)
        sin_y = math.sin(self.drone_yaw)

        for va in v_angles:
            for ha in h_angles:
                dx_local = math.sin(ha)
                dy_local = math.cos(ha) * math.sin(va)  # FIX Bug1: was missing cos(ha) → non-unit-length rays
                dz_local = math.cos(ha) * math.cos(va)

                rx = cos_y * dz_local - sin_y * dx_local
                ry = sin_y * dz_local + cos_y * dx_local
                rz = -dy_local

                direction = (rx, ry, rz)

                closest_t = self.max_range
                hit_found = False

                for obs in self.obstacles:
                    t = ray_box_intersection(origin, direction, obs)
                    if t is not None and 0.2 <= t <= closest_t:
                        closest_t = t
                        hit_found = True

                if hit_found:
                    # Calculate exact 3D world coordinate on outer wall skin (0.00 mm penetration)
                    px = self.drone_x + rx * closest_t
                    py = self.drone_y + ry * closest_t
                    pz = self.drone_z + rz * closest_t
                    points.append([px, py, pz])

        return points


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDroneMaze()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
