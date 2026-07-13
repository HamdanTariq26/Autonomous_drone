"""
config/constants.py

Every ROS2 topic/node name, filesystem path, and shared numeric
constant used across the tello_autonomy package, in one place. YAML
files in this same folder (tello_cam.yaml, ros_params.yaml,
planner_params.yaml) hold config that's naturally data - camera
intrinsics, per-node ROS params, planner tuning values. This file holds
what code actually imports and references directly by name.

Nothing outside this file should hardcode a topic name string or a
path. If a topic name or path ever needs to change, it changes here
once.
"""

import os

# ----------------------------------------------------------------------
# Filesystem paths
# ----------------------------------------------------------------------
HOME_DIR = os.path.expanduser("~")

# CSV/trajectory files written by the C++ SLAM node.
LIVE_KEYFRAME_CSV = os.path.join(HOME_DIR, "live_sparse_map_points.csv")
FINAL_KEYFRAME_CSV = os.path.join(HOME_DIR, "SparseMapPoints.csv")
KEYFRAME_TRAJECTORY_TXT = os.path.join(HOME_DIR, "KeyFrameTrajectory.txt")

# Where drone_interface saves every frame (for later depth/scale
# matching by timestamp), and where any cleanup logic deletes from.
SAVE_FRAMES_DIR = "frame_images"


# ----------------------------------------------------------------------
# ROS2 topic names - must match whatever the C++ SLAM node and any
# ROS2 nodes in this package actually publish/subscribe to.
# ----------------------------------------------------------------------
TOPIC_EXPERIMENT_SETTINGS = "/mono_py_driver/experiment_settings"
TOPIC_EXP_SETTINGS_ACK = "/mono_py_driver/exp_settings_ack"
TOPIC_IMG_MSG = "/mono_py_driver/img_msg"
TOPIC_TIMESTEP_MSG = "/mono_py_driver/timestep_msg"
TOPIC_KEYFRAME_TIMESTAMPS = "/mono_py_driver/keyframe_timestamps"
TOPIC_MAP_TOPOLOGY_CHANGED = "/mono_py_driver/map_topology_changed"

# Reserved for future layers (occupancy_map / exploration / search /
# goals) - not yet published by anything, named here ahead of time so
# there's one place to look when those layers get built.
TOPIC_OCCUPANCY_GRID = "/tello_autonomy/occupancy_grid"
TOPIC_FRONTIER_GOALS = "/tello_autonomy/frontier_goals"
TOPIC_PLANNED_PATH = "/tello_autonomy/planned_path"
TOPIC_MISSION_STATUS = "/tello_autonomy/mission_status"


# ----------------------------------------------------------------------
# ROS2 node / package names
# ----------------------------------------------------------------------
ROS2_PACKAGE_NAME = "ros2_orb_slam3"
ROS2_SLAM_NODE_EXECUTABLE = "mono_node_cpp"
ROS2_SLAM_NODE_NAME_ARG = "mono_slam_cpp"


# ----------------------------------------------------------------------
# Drone / camera constants
# ----------------------------------------------------------------------
DEFAULT_SETTINGS_NAME = "TelloCam"     # matches tello_cam.yaml
DEFAULT_MAX_PUB_RATE_HZ = 15.0
DEFAULT_MANUAL_SPEED_CMS = 50
TELLO_VIDEO_BITRATE = "BITRATE_5MBPS"  # matches Tello.BITRATE_5MBPS in djitellopy

CAMERA_WIDTH = 960
CAMERA_HEIGHT = 720


# ----------------------------------------------------------------------
# Frame cleanup constants
# ----------------------------------------------------------------------
KEYFRAME_MATCH_TOLERANCE_SEC = 0.01
DELETION_GRACE_PERIOD_SEC = 3.0
CLEANUP_INTERVAL_SEC = 2.0


# ----------------------------------------------------------------------
# SLAM / scale-factor constants
# ----------------------------------------------------------------------
MIN_KEYFRAMES_FOR_SCALE_FACTOR = 15
PERIODIC_RECALIBRATION_SECONDS = 15

# --- NEW (added for perception/scale_factor_manager.py) ---
# How many of the MOST RECENT keyframes (by keyframe_id) for a given
# map_id to use on each (re)computation - both the first computation
# and every periodic one after it. Deliberately NOT "every keyframe
# from the last PERIODIC_RECALIBRATION_SECONDS seconds" - a keyframe
# count is used instead of a time window, so behavior doesn't change
# based on how fast keyframes happen to be arriving at the moment.
# Should generally be >= MIN_KEYFRAMES_FOR_SCALE_FACTOR.
SCALE_FACTOR_RECENT_KEYFRAME_COUNT = 30
 
# Points with SLAM depth below this are skipped when computing ratios -
# avoids divide-by-near-zero ratio blowups (ported from the old
# compute_scale_factor.py CLI script's --min_slam_depth default).
MIN_SLAM_DEPTH = 1e-3
 
 
# ----------------------------------------------------------------------
# Depth Anything V2 (metric depth model) constants
# ----------------------------------------------------------------------
# NEW - the Depth-Anything-V2 repo lives outside this package and is
# added to sys.path (see perception/depth/depth_anything_v2.py) rather
# than pip-installed. This is the metric_depth subfolder specifically -
# that's where the `depth_anything_v2` importable package lives for
# the metric-depth variant of the repo.
# NEW - the Depth-Anything-V2 repo lives outside this package and is
# added to sys.path (see perception/depth/depth_anything_v2.py) rather
# than pip-installed. This is the metric_depth subfolder specifically -
# that's where the `depth_anything_v2` importable package lives for
# the metric-depth variant of the repo.
#
# CHANGED: computed relative to this file's own location instead of a
# hardcoded absolute path. The previous version was hardcoded to
# /home/hamdan/autonomous_drone/... which (a) breaks on any other
# machine/username, and (b) was already missing the ros2_test path
# segment, pointing at a directory that doesn't exist. Computing this
# from __file__ means it survives the whole project folder being
# renamed or moved again, the same class of bug that broke
# common.hpp's packagePath earlier in this project.
_TELLO_AUTONOMY_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEPTH_ANYTHING_V2_REPO_PATH = os.path.join(
    _TELLO_AUTONOMY_ROOT, "perception", "depth", "Depth-Anything-V2", "metric_depth"
)
 
# NEW - adjust the filename here if your checkpoint differs; this
# matches the old CLI script's example invocation (vits encoder,
# hypersim/indoor-trained checkpoint).
DEPTH_ANYTHING_V2_CHECKPOINT_PATH = os.path.join(
    DEPTH_ANYTHING_V2_REPO_PATH, "checkpoints", "depth_anything_v2_metric_hypersim_vits.pth"
)
 
DEPTH_ANYTHING_V2_ENCODER = "vits"     # NEW - "vits" for CPU-practical inference
DEPTH_ANYTHING_V2_MAX_DEPTH = 20.0     # NEW - meters; 20.0 for indoor/hypersim, 80.0 for outdoor/vkitti
DEPTH_ANYTHING_V2_DEVICE = "cpu"       # NEW - "cuda" if/when this runs on a GPU-equipped machine

# NEW - per-frame pose + tracked-points stream, published by the C++
# SLAM node every tracked frame (not just at keyframe rate like the
# CSV). Values on these topics are in ORB-SLAM3's arbitrary SLAM units,
# NOT metric - see perception/live_scaler.py, which subscribes to
# these, multiplies by the current per-map_id scale factor, and
# republishes the _METRIC versions below. Nothing outside
# live_scaler.py should ever subscribe to these RAW topics directly.
TOPIC_CURRENT_POSE_RAW = "/tello_autonomy/current_pose_raw"
TOPIC_CURRENT_POINTS_RAW = "/tello_autonomy/current_points_raw"

# NEW - metric-scale versions of the above, republished by
# perception/live_scaler.py once a scale factor exists for the
# relevant map_id. These are what occupancy_map/ and later layers
# should actually subscribe to - never the _RAW topics.
TOPIC_CURRENT_POSE_METRIC = "/tello_autonomy/current_pose_metric"
TOPIC_CURRENT_POINTS_METRIC = "/tello_autonomy/current_points_metric"

# NEW - both PoseStamped and PointCloud2 messages on the topics above
# carry their map_id by setting header.frame_id to this prefix + the
# map_id (e.g. "slam_map_3"), rather than via a custom message field -
# avoids needing a new custom .msg/interfaces package. live_scaler.py
# parses map_id back out using this same prefix - keep both in sync.
SLAM_MAP_FRAME_ID_PREFIX = "slam_map_"

# NEW - topics published by PublishLiveMapData() (C++ side), replacing
# the old live CSV. One message per active map_id, per periodic tick.
TOPIC_KEYFRAME_POINTS = "/tello_autonomy/keyframe_points"
TOPIC_TRAJECTORY = "/tello_autonomy/trajectory"

# NEW - the C++ node's periodic publish timer runs every ~1s
# (LiveCsvTimer_callback). If no keyframe_points message has been
# received for a map_id in longer than this, treat it as stale/merged
# away - generous margin (3x) over the expected ~1s cadence to absorb
# normal jitter without false-pruning an active map.
MAP_DATA_STALE_AFTER_SEC = 3.0


# NEW - the depth-inference worker process's niceness (relative to the
# default 0). Positive = lower OS scheduling priority, so the kernel
# favors flight control / ROS2 callbacks over this process whenever
# both want the CPU at the same time.
DEPTH_WORKER_NICE_VALUE = 10

# NEW - how often scale_factor_manager's result-poll timer checks the
# worker process's result queue. Cheap non-blocking drain, so this can
# be fairly frequent without any real cost.
DEPTH_WORKER_RESULT_POLL_INTERVAL_SEC = 0.5
