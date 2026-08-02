"""
perception/depth_backprojection.py

Turns a dense Depth-Anything-V2 metric depth map (the SAME output
perception/scale_factor.py already computes and then discards after one
ratio) into a 3D point cloud, using real intrinsics from
perception/camera_intrinsics.py - not a nominal-FOV approximation.

Points are returned in the CAMERA frame (X=right, Y=down, Z=forward),
matching this project's established SLAM convention (see
goals/trajectory_tracker.py's coordinate-frame docstring) - no axis
remapping needed. Transforming these into the world/map frame requires the
capturing keyframe's pose, which this module deliberately does NOT handle -
that's a ROS2-side concern (subscribing to TOPIC_TRAJECTORY and matching by
timestamp), kept out of this pure-math module the same way
perception/scale_factor.py stays free of ROS2 and torch.

WHY STRIDE + DEPTH BOUNDS + EDGE REJECTION
--------------------------------------------
- A full-resolution 960x720 depth map back-projected at every pixel is
  ~700,000 points per keyframe - useless for an OctoMap insertion and a
  serialization/bandwidth problem on top of it. BACKPROJECT_STRIDE_PX
  samples a sparse grid instead, tuned to land in the same rough order of
  magnitude as the simulator's synthetic 40x30 ray scan.
- BACKPROJECT_MIN_DEPTH_M / BACKPROJECT_MAX_DEPTH_M drop points too close
  (lens/near-field noise) or too far (monocular depth nets get
  increasingly unreliable at range) to trust.
- BACKPROJECT_MAX_DEPTH_GRADIENT_M rejects "flying pixels" - the classic
  monocular-depth artifact where pixels straddling an object's silhouette
  against the background get an interpolated depth partway between the
  two, producing a phantom point floating in empty space between the
  object and the wall behind it. Comparing each sampled pixel against its
  right/down neighbor and dropping it if the jump is too large is a cheap,
  effective filter for this.
"""

import numpy as np

from config import constants


def backproject_depth_to_camera_points(
    depth_m,
    intrinsics,
    stride_px=None,
    min_depth_m=None,
    max_depth_m=None,
    max_depth_gradient_m=None,
):
    """
        depth_m: (H, W) numpy array of metric depth in meters, as returned
            by perception.depth.depth_anything_v2.infer_metric_depth(). Its
            resolution must match `intrinsics` (Depth-Anything-V2's
            infer_image already interpolates its output back to the
            original input image size, so this holds automatically as long
            as the same image was used for both).
        intrinsics: a perception.camera_intrinsics.CameraIntrinsics
            instance (see load_intrinsics()).
        stride_px, min_depth_m, max_depth_m, max_depth_gradient_m: default
            to the matching constants.BACKPROJECT_* values - override only
            for one-off experimentation.

        Returns an (N, 3) numpy array of camera-frame points
        (X=right, Y=down, Z=forward, meters). N depends entirely on how
        many sampled pixels survive the depth-bounds and gradient filters -
        not a fixed count.
    """
    stride_px = stride_px if stride_px is not None else constants.BACKPROJECT_STRIDE_PX
    min_depth_m = min_depth_m if min_depth_m is not None else constants.BACKPROJECT_MIN_DEPTH_M
    max_depth_m = max_depth_m if max_depth_m is not None else constants.BACKPROJECT_MAX_DEPTH_M
    max_depth_gradient_m = (
        max_depth_gradient_m if max_depth_gradient_m is not None
        else constants.BACKPROJECT_MAX_DEPTH_GRADIENT_M
    )

    h, w = depth_m.shape
    vs = np.arange(0, h - 1, stride_px)  # h-1/w-1 so the neighbor-gradient check below never runs off the edge
    us = np.arange(0, w - 1, stride_px)
    grid_v, grid_u = np.meshgrid(vs, us, indexing="ij")

    z = depth_m[grid_v, grid_u]
    z_right = depth_m[grid_v, np.minimum(grid_u + 1, w - 1)]
    z_down = depth_m[np.minimum(grid_v + 1, h - 1), grid_u]

    valid = (z > min_depth_m) & (z < max_depth_m)
    gradient = np.maximum(np.abs(z - z_right), np.abs(z - z_down))
    valid &= gradient < max_depth_gradient_m

    grid_u, grid_v, z = grid_u[valid], grid_v[valid], z[valid]
    if z.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    x = (grid_u - intrinsics.cx) * z / intrinsics.fx
    y = (grid_v - intrinsics.cy) * z / intrinsics.fy

    return np.stack([x, y, z], axis=-1).astype(np.float64)
