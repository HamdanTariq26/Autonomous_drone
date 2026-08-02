"""
perception/pose_transform.py

Small, pure-numpy geometry helper - no ROS2, no torch, same "pure math"
character as perception/scale_factor.py and perception/depth_backprojection.py.
Turns a set of camera-frame 3D points (as produced by
perception.depth_backprojection.backproject_depth_to_camera_points) into
world-frame points, given the capturing keyframe's pose.

CRITICAL UNIT NOTE
------------------
Rotation is scale-invariant - a keyframe's orientation is the same whether
you're working in ORB-SLAM3's raw arbitrary units or real meters, so the raw
SLAM orientation can be used directly.

Translation is NOT scale-invariant. TOPIC_TRAJECTORY (where keyframe poses
come from) publishes RAW, unscaled positions - same units as
TOPIC_CURRENT_POSE_RAW. Our depth-backprojected points are already in real
meters (Depth-Anything-V2's metric output). Combining metric camera points
with a raw-unit translation would silently place the entire point cloud at
the wrong distance from the camera - not a crash, just quietly wrong data
in the occupancy map. Callers MUST scale the raw translation by that
map_id's current scale factor (the same one perception.live_scaler.LiveScaler
already applies to everything else) before calling
transform_camera_points_to_world() - see perception/scale_factor_manager.py's
_publish_dense_depth_points() for where that happens.
"""

import numpy as np


def _quaternion_to_rotation_matrix(x, y, z, w):
    """Standard unit-quaternion -> 3x3 rotation matrix conversion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_camera_points_to_world(points_cam, orientation, translation_m):
    """
        points_cam: (N, 3) numpy array, camera frame (X=right, Y=down,
            Z=forward - see perception/depth_backprojection.py).
        orientation: anything with .x/.y/.z/.w (e.g. a
            geometry_msgs.msg.Quaternion) - the keyframe's RAW SLAM
            orientation, used as-is (rotation is scale-invariant).
        translation_m: an (x, y, z) tuple/list/array giving the keyframe's
            position in REAL METERS - i.e. the raw SLAM position ALREADY
            MULTIPLIED by the current scale factor for that map_id. Passing
            a still-raw translation here is the one mistake this function
            cannot detect for you.

        Returns an (N, 3) numpy array of world-frame points in meters.
    """
    if points_cam.shape[0] == 0:
        return points_cam

    rotation = _quaternion_to_rotation_matrix(
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    translation = np.asarray(translation_m, dtype=np.float64)
    return points_cam @ rotation.T + translation
