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

# NEW - raw BGR frames published in-memory instead of saved to disk.
# Replaces SAVE_FRAMES_DIR + FrameCleanupNode entirely (pipeline_audit.md
# finding: cv2.imwrite at 30Hz + FrameCleanupNode's periodic glob/delete
# were both competing with FFMPEG's decode thread for the GIL, causing
# stochastic SLAM-viewer lag that would build up then sometimes drain).
TOPIC_RECENT_FRAME = "/tello_autonomy/recent_frame"

# Ring buffer bound in ScaleFactorManager: max age of a frame kept in
# memory before it's evicted, regardless of whether it was ever used.
# Must comfortably exceed the worst-case lookup window: a keyframe's
# timestamp can lag PERIODIC_RECALIBRATION_SECONDS behind "now" before
# it's ever matched against a frame, plus KEYFRAME_MATCH_TOLERANCE_SEC
# slop. 90s gives margin above PERIODIC_RECALIBRATION_SECONDS (15s)
# without accumulating unbounded memory.
RECENT_FRAME_BUFFER_MAX_AGE_SEC = 90.0

# Hard cap on buffer size regardless of age - the actual memory bound.
# At CAMERA_WIDTH x CAMERA_HEIGHT x 3 bytes (BGR8, no compression) this
# is the real ceiling: 960x720x3 ~= 2MB/frame, so 150 frames ~= 300MB
# worst case. Age-based eviction (above) will normally keep the buffer
# far smaller than this; this cap is the backstop if publish rate ever
# spikes relative to eviction/consumption.
RECENT_FRAME_BUFFER_MAX_COUNT = 150

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
DEFAULT_MAX_PUB_RATE_HZ = 30.0
DEFAULT_MANUAL_SPEED_CMS = 50
TELLO_VIDEO_BITRATE = "BITRATE_5MBPS"  # matches Tello.BITRATE_5MBPS in djitellopy

CAMERA_WIDTH = 960
CAMERA_HEIGHT = 720


# ----------------------------------------------------------------------
# Frame matching constants
# ----------------------------------------------------------------------
KEYFRAME_MATCH_TOLERANCE_SEC = 0.01


# ----------------------------------------------------------------------
# SLAM / scale-factor constants
# ----------------------------------------------------------------------
# Minimum keyframes before the first scale computation is attempted.
MIN_KEYFRAMES_FOR_SCALE_FACTOR = 15

# How often the scale factor is periodically recomputed (seconds).
PERIODIC_RECALIBRATION_SECONDS = 15

# --- NEW (added for perception/scale_factor_manager.py) ---
# How many of the MOST RECENT keyframes (by keyframe_id) for a given
# map_id to use on each (re)computation.
#
# pipeline_audit.md Finding #4: with DEPTH_INFERENCE_STRIDE=1 (below),
# every one of these keyframes actually runs inference - so the
# effective sample size feeding each recompute's median is this number
# directly (not divided by stride, the way an earlier revision's
# comment here used to claim - that comment was stale relative to the
# live values and has been corrected rather than left to mislead the
# next reader). 30 keyframes is enough that a handful of motion-blurred
# or otherwise bad ratios (see MIN_SLAM_DEPTH and
# SCALE_RATIO_MIN/SCALE_RATIO_MAX below) can't dominate the median the
# way ~4 samples could.
# Must be >= MIN_KEYFRAMES_FOR_SCALE_FACTOR.
SCALE_FACTOR_RECENT_KEYFRAME_COUNT = 30

# Points with SLAM depth below this are skipped when computing ratios -
# avoids divide-by-near-zero ratio blowups.
#
# pipeline_audit.md Finding #5: 1mm (the old default, ported from an
# even older CLI script's --min_slam_depth) is near-degenerate
# triangulation noise, not a real indoor measurement - nothing in a
# typical room legitimately triangulates that close to the camera.
# ratio = metric_depth / slam_depth blows up as slam_depth -> 0, so a
# handful of these can badly skew even a median if enough cluster
# together. Raised to 0.2m: comfortably below any real indoor obstacle
# distance while well clear of the triangulation noise floor.
MIN_SLAM_DEPTH = 0.2

# ----------------------------------------------------------------------
# Live point-cloud depth filtering (perception/live_scaler.py)
# ----------------------------------------------------------------------
# Points published on TOPIC_CURRENT_POINTS_RAW have no depth filter applied
# on the C++ side (PublishCurrentPoseAndPoints() only rejects points behind
# the camera, camPos.z() <= 0). Monocular triangulation is numerically
# unstable for points with very small parallax between frames - this shows
# up most often for points near the epipole (i.e. roughly along the
# direction of travel), where small pixel noise can collapse the estimated
# depth to near-zero. Those near-zero-depth points land almost on top of
# the camera and get inserted into the OctoMap as a voxel cluster right in
# front of the drone during forward flight.
#
# Mirrors MIN_SLAM_DEPTH's rationale (perception/scale_factor.py), but
# applied here in METRIC units (post-scale) since this is the earliest
# point in the pipeline where the points are already in meters. Filtering
# here - rather than in common.cpp, which only has raw SLAM units - keeps
# the threshold tunable/inspectable in real-world units without a rebuild.
MIN_LIVE_POINT_DEPTH_M = 0.20

# pipeline_audit.md Finding #6: bounds on the raw (metric_depth /
# slam_depth) ratio itself, applied in
# scale_factor.compute_scale_factor_from_ratios() before taking the
# median. A plain median has no protection if more than ~50% of a
# cycle's points are corrupted (e.g. by motion blur on a fast yaw - see
# rrt.cpp's own yaw penalty comments for why that's a real, expected
# occurrence, not a hypothetical). Any single ratio outside this range
# implies either a near-zero SLAM depth that slipped past
# MIN_SLAM_DEPTH, a spurious near-zero Depth-Anything prediction, or a
# genuine outlier - none of which should be allowed to pull the median.
# 0.05x-20x is a generous band (a real scale factor for this rig should
# sit well inside it); the intent is to catch only the worst garbage,
# not to second-guess plausible-but-unusual values.
SCALE_RATIO_MIN = 0.05
SCALE_RATIO_MAX = 20.0

# Subsampling stride for Depth Anything inference inside each compute
# cycle. stride=2 means only 1 in every 2 matched keyframes actually
# runs the model; stride=1 disables subsampling (all frames inferred).
# Adjacent SLAM keyframes share ~80-90% of their visible scene, so
# skipping every other one has negligible effect on the median ratio
# while halving inference time. Raise to 2 or 3 if CPU is too slow to
# keep SCALE_FACTOR_RECENT_KEYFRAME_COUNT keyframes within
# PERIODIC_RECALIBRATION_SECONDS - but note that doing so shrinks the
# effective sample size feeding the median (see Finding #4 above), so
# prefer raising PERIODIC_RECALIBRATION_SECONDS first if there's slack
# for a slower recompute cadence instead.
DEPTH_INFERENCE_STRIDE = 1


# ----------------------------------------------------------------------
# Scale-factor quarantine (pipeline_audit.md, Finding #3)
# ----------------------------------------------------------------------
# Every PERIODIC_RECALIBRATION_SECONDS, ScaleFactorManager can replace
# current_scale_factors[map_id] with a value that differs sharply from
# the last one (calibrate_scale_factor.py's own docstring cites a real
# example: "suggested_scale = current * 1.35"). occupancy_map_cpp's
# OcTree already contains points inserted under the OLD scale by the
# time that happens - there is no mechanism reconciling old insertions
# with a corrected scale, so a large jump silently smears/duplicates
# geometry in the live map with no visible error.
#
# QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD: if a newly computed scale
# factor differs from the currently active one by more than this
# fractional amount (0.15 = 15%), ScaleFactorManager does not adopt it
# immediately. Instead the candidate is held in "quarantine" and only
# adopted once QUARANTINE_CONFIRMATIONS_REQUIRED consecutive recompute
# cycles agree with it (within this same threshold of each other) -
# this distinguishes a genuine, sustained correction (e.g. the drone's
# actual scale really did drift, or the first estimate was simply bad)
# from a single noisy cycle's outlier result. Small, gradual updates
# under this threshold are still adopted immediately, same as before -
# this only guards against sharp, sudden jumps.
QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD = 0.15

# How many consecutive recompute cycles must agree (each within
# QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD of the previous candidate, not
# of the original active value) before a quarantined jump is adopted.
# 2 means "confirm once" - the jump must repeat rather than being a
# one-off, at a cost of one extra PERIODIC_RECALIBRATION_SECONDS of
# delay before a real correction takes effect.
QUARANTINE_CONFIRMATIONS_REQUIRED = 2

# How long a quarantined candidate from an already-smoothed source
# (ext_tof / internal_tof) must persist within
# QUARANTINE_SCALE_JUMP_RATIO_THRESHOLD of itself before being adopted.
# These sources return the same stable float on every poll once locked
# on, so the "must look like a new measurement" check used for depth
# never fires for them - a time-based hold is used instead.
QUARANTINE_MIN_HOLD_SEC = 3.0

 
 
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
 
# ----------------------------------------------------------------------
# Camera calibration (read from ORB-SLAM3's own settings file - never
# duplicated/hardcoded in Python; see perception/camera_intrinsics.py)
# ----------------------------------------------------------------------
# NOTE: mirrors the same __file__-relative pattern already used above for
# DEPTH_ANYTHING_V2_REPO_PATH, for the same reason common.hpp's hardcoded
# packagePath was flagged as fragile - this survives the project folder
# being renamed/moved.
_AUTONOMOUS_DRONE_ROOT = os.path.dirname(_TELLO_AUTONOMY_ROOT)
ORBSLAM3_CAMERA_YAML_PATH = os.path.join(
    _AUTONOMOUS_DRONE_ROOT, "ros2_test", "src", "ros2_orb_slam3",
    "orb_slam3", "config", "Monocular", f"{DEFAULT_SETTINGS_NAME}.yaml",
)

# ----------------------------------------------------------------------
# Dense depth back-projection (perception/depth_backprojection.py) -
# turns Depth-Anything-V2's per-keyframe depth map into a 3D point cloud
# instead of discarding it after the scale-factor ratio.
# ----------------------------------------------------------------------
BACKPROJECT_STRIDE_PX = 20          # sample every Nth pixel in both axes
BACKPROJECT_MIN_DEPTH_M = 0.2       # drop near-field/lens noise
BACKPROJECT_MAX_DEPTH_M = 6.0       # drop far-range points the depth net can't be trusted at
BACKPROJECT_MAX_DEPTH_GRADIENT_M = 0.5  # reject "flying pixel" edge artifacts

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
TOPIC_DENSE_POINTS_METRIC = "/tello_autonomy/dense_points_metric"

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

# Published by perception/live_scaler.py whenever it detects a map_id
# transition on the raw pose stream. Carries the alignment transform
# (new_map_raw_frame -> old_map_metric_frame) computed in RAW SLAM units
# (both anchor poses share the same arbitrary scale, avoiding the bug where
# comparing two already-scaled metric poses across different scale factors
# produces a meaningless offset). occupancy_map_cpp subscribes to this
# instead of computing alignment itself - it never sees raw poses or
# per-map scale factors so cannot do the math correctly on its own.
# Message type: tello_autonomy_msgs/msg/MapAlignment
TOPIC_MAP_ALIGNMENT = "/tello_autonomy/map_alignment"

# Sanity bounds used by live_scaler.py when deciding whether to accept a
# computed alignment transform. Both conditions must hold independently -
# the old OR-clause bypass (small offset passes even over a long gap) was
# incorrect because a small estimated offset over a long gap tells you
# nothing about whether the anchor is trustworthy.
MAX_ALIGNMENT_TRANSLATION_M = 4.0   # max acceptable metric offset between anchors
                                    # (raised from 2.0: drone can travel 3-4m during a recovery
                                    #  sweep before ORB-SLAM3 re-initialises a new map)
MAX_ALIGNMENT_GAP_SEC = 25.0        # max acceptable wall-clock gap waiting for new map scale factor

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


# ----------------------------------------------------------------------
# Goals Layer / Autonomous Navigation constants
# ----------------------------------------------------------------------
# Rate at which the mission controller checks trajectory error and 
# sends velocity commands to the drone.
GOALS_LOOP_RATE_HZ = 10.0

# Acceptable distance to a waypoint to consider it "reached" and pop
# it from the path queue. Increased to 0.35m so the drone doesn't slow
# down to a crawl at every intermediate point.
WAYPOINT_ACCEPTANCE_RADIUS_M = 0.35

# Distance at which the drone starts braking (linearly scaling speed to
# zero). Must be > WAYPOINT_ACCEPTANCE_RADIUS_M. Set larger for faster
# max speeds to give the drone enough room to decelerate.
BRAKING_RADIUS_M = 0.4

# Maximum allowable velocities (0-100) for autonomous flight.
# Increased to 60 for much faster flight.
MAX_AUTO_SPEED_XY = 60
MAX_AUTO_SPEED_Z  = 40
MAX_AUTO_SPEED_YAW = 30

# Proportional gain for the position P-controller.
# Increased significantly so the drone accelerates hard toward waypoints.
XY_P_GAIN  = 150.0
Z_P_GAIN   = 100.0
YAW_P_GAIN = 15.0


# ----------------------------------------------------------------------
# ToF-based scale estimation (alternative/complement to Depth-Anything-V2)
# ----------------------------------------------------------------------

# New ROS2 topics published by middleware/telemetry_bridge.py.
TOPIC_TOF_HEIGHT_CM = "/tello_autonomy/tof_height_cm"
TOPIC_BARO_HEIGHT_M = "/tello_autonomy/baro_height_m"

# How often telemetry_bridge polls/publishes ToF + barometer (Hz).
TOF_POLL_HZ = 10.0

# ToF validity bounds (cm).  Readings outside this open interval are
# rejected by perception/tof_scale_estimator.py.
#   Lower bound (35 cm): matches the empirically observed near-field
#   clamp (~10 cm output when true distance is under ~30 cm) plus a
#   small safety margin.  Treating those readings as "valid" would
#   corrupt the rolling median with systematically low scale factors.
#   Upper bound (800 cm): the sensor's real usable range tops out well
#   under this; anything near the ~8192 out-of-range sentinel is also
#   excluded regardless of the exact sentinel value.
TOF_MIN_VALID_CM = 35
TOF_MAX_VALID_CM = 800

# Rolling window size (valid samples) kept per map_id in
# tof_scale_estimator.py for smoothing and noise checks.
TOF_SCALE_ROLLING_WINDOW = 15

# Auto-mode primary gate: if the raw ToF readings (cm) in the rolling
# window have a standard deviation above this, the ToF estimate is
# considered unstable and Depth-Anything-V2 is preferred instead.
# Exposed as a constant so it can be tuned without touching code;
# 15 cm is a reasonable starting point but should be revisited once
# you have real flight data (15 cm noise means something different at
# 50 cm AGL vs 3 m AGL).
TOF_SCALE_STDDEV_THRESHOLD_CM = 15.0

# Barometer vs ToF disagreement threshold (cm) -- INFORMATIONAL /
# LOGGED ONLY, never a hard auto-mode trigger.  Indoor barometers
# drift from HVAC and door pressure changes by tens of centimetres,
# so a disagreement here is as likely to mean the barometer is wrong
# as the ToF; logging it keeps the data visible without letting a
# noisy barometer override a perfectly good ToF reading.
BARO_TOF_DISAGREEMENT_WARN_CM = 75.0
BARO_WARN_THROTTLE_SEC = 5.0

# Runtime scale-source selector.  Changed live via 'm' key in
# ManualControl -> ScaleFactorManager.cycle_scale_source_mode().
#   "tof"   -- always use the ToF ratio; hold last valid value when invalid.
#   "depth" -- Depth-Anything-V2 exclusively (pre-existing behaviour).
#   "auto"  -- prefer ToF when valid and stable (std_dev below threshold);
#              fall back to Depth-Anything otherwise.
SCALE_SOURCE_MODE = "auto"

# ----------------------------------------------------------------------
# Extension-kit ToF (EXT tof? command, mm-precision, ~1.2m range)
# ----------------------------------------------------------------------
TOPIC_EXT_TOF_DISTANCE_MM = "/tello_autonomy/ext_tof_distance_mm"
TOPIC_EXT_TOF_VALID = "/tello_autonomy/ext_tof_valid"
EXT_TOF_BRIDGE_POLL_HZ = 5.0   # topic publish rate; underlying sensor poll is its own thread's pace

