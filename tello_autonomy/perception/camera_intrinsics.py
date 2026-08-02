"""
perception/camera_intrinsics.py

Loads the SAME camera calibration ORB-SLAM3 itself uses, straight from the
.yaml file already sitting in your ORB-SLAM3 config folder
(config.constants.ORBSLAM3_CAMERA_YAML_PATH, e.g. TelloCam.yaml) - never
hardcoded in Python. If you ever recalibrate the lens or swap cameras, this
picks it up automatically the next time it's loaded; nothing here needs to
change.

WHY THE MANUAL PARSING (not just yaml.safe_load directly)
------------------------------------------------------------
OpenCV's FileStorage YAML format starts every file with a header line like:

    %YAML:1.0

That is NOT valid standard-YAML directive syntax (real YAML wants
"%YAML 1.0" - a space, not a colon), so a plain `yaml.safe_load()` call on
one of these files raises a scanner/reader error before it ever gets to the
actual key/value pairs. The fix is simple: strip that first line before
handing the rest to PyYAML, since everything after it (for this project's
config - plain scalars, no OpenCV !!opencv-matrix tags) is completely
ordinary YAML.

WHAT THIS GIVES YOU
--------------------
- A cached, singleton-style CameraIntrinsics object (mirrors the caching
  pattern already used in perception/depth/depth_anything_v2.py) with:
    fx, fy, cx, cy            - pinhole projection parameters
    k1, k2, p1, p2            - radial/tangential distortion
    width, height             - image resolution
    camera_matrix (3x3 np array)   - ready for cv2.undistort / manual math
    dist_coeffs (np array [k1,k2,p1,p2])
- back_project_pixel(u, v, depth_m) - single-pixel pixel->3D helper, in the
  CAMERA frame (X=right, Y=down, Z=forward - matches this project's
  established SLAM convention, see goals/trajectory_tracker.py's docstring,
  so no axis remapping is needed downstream).
"""

import os

import numpy as np
import yaml

from config import constants


def _strip_opencv_yaml_header(raw_text):
    """
        Removes the leading '%YAML:1.0' (and, if present, a following
        '---' document-start marker) OpenCV FileStorage writes, which
        PyYAML's directive parser rejects. Any other content is left
        completely untouched.
    """
    lines = raw_text.splitlines()
    cleaned = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("%YAML"):
            continue
        if stripped == "---":
            continue
        cleaned.append(line)
    return "\n".join(cleaned)


class CameraIntrinsics:
    def __init__(self, fx, fy, cx, cy, k1, k2, p1, p2, width, height):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.k1, self.k2, self.p1, self.p2 = k1, k2, p1, p2
        self.width, self.height = width, height

        self.camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
        )
        # OpenCV's 4-coefficient (no k3) radial-tangential convention:
        # (k1, k2, p1, p2)
        self.dist_coeffs = np.array([k1, k2, p1, p2], dtype=np.float64)

    def back_project_pixel(self, u, v, depth_m):
        """
            Single-pixel pinhole back-projection: image coords + metric
            depth -> a 3D point in the CAMERA frame (X=right, Y=down,
            Z=forward). This is the exact same math
            perception/depth_backprojection.py applies across a whole
            depth map - exposed here too since callers occasionally just
            need one point (e.g. cross-checking a single ORB keypoint).
        """
        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        return x, y, depth_m

    def __repr__(self):
        return (
            f"CameraIntrinsics(fx={self.fx:.3f}, fy={self.fy:.3f}, "
            f"cx={self.cx:.3f}, cy={self.cy:.3f}, "
            f"k1={self.k1:.6f}, k2={self.k2:.6f}, "
            f"p1={self.p1:.6f}, p2={self.p2:.6f}, "
            f"{self.width}x{self.height})"
        )


_intrinsics = None  # module-level singleton, same pattern as depth_anything_v2.py


def load_intrinsics(yaml_path=None, force_reload=False):
    """
        Loads (or returns the already-loaded) CameraIntrinsics from
        yaml_path (defaults to config.constants.ORBSLAM3_CAMERA_YAML_PATH).
        Cached after first successful load - pass force_reload=True if you
        genuinely changed the file on disk mid-run (e.g. interactive
        recalibration) and need it re-read.
    """
    global _intrinsics
    if _intrinsics is not None and not force_reload:
        return _intrinsics

    yaml_path = yaml_path or constants.ORBSLAM3_CAMERA_YAML_PATH
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(
            f"Camera calibration file not found at '{yaml_path}'. "
            "Check config.constants.ORBSLAM3_CAMERA_YAML_PATH matches your "
            "actual ORB-SLAM3 settings file location."
        )

    with open(yaml_path, "r") as f:
        raw_text = f.read()
    data = yaml.safe_load(_strip_opencv_yaml_header(raw_text))

    required = ["Camera1.fx", "Camera1.fy", "Camera1.cx", "Camera1.cy",
                "Camera1.k1", "Camera1.k2", "Camera1.p1", "Camera1.p2",
                "Camera.width", "Camera.height"]
    missing = [key for key in required if key not in data]
    if missing:
        raise KeyError(
            f"Camera calibration file '{yaml_path}' is missing expected "
            f"key(s): {missing}. Nothing was cached - fix the file and "
            f"reload rather than silently falling back to a guess."
        )

    _intrinsics = CameraIntrinsics(
        fx=float(data["Camera1.fx"]), fy=float(data["Camera1.fy"]),
        cx=float(data["Camera1.cx"]), cy=float(data["Camera1.cy"]),
        k1=float(data["Camera1.k1"]), k2=float(data["Camera1.k2"]),
        p1=float(data["Camera1.p1"]), p2=float(data["Camera1.p2"]),
        width=int(data["Camera.width"]), height=int(data["Camera.height"]),
    )
    return _intrinsics
