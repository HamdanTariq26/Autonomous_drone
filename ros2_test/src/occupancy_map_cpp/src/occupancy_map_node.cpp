#include "occupancy_map_cpp/occupancy_map_node.hpp"

#include <chrono>
#include <cmath>

#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace occupancy_map_cpp
{

namespace
{
// Matches config.constants.SLAM_MAP_FRAME_ID_PREFIX on the Python side.
// Kept as a local constant rather than a parameter - this is a wire-format
// convention shared with the Python nodes (see live_scaler.py /
// scale_factor_manager.py's own _parse_map_id_from_frame_id helpers),
// not something any single node's config should be free to change alone.
constexpr const char * kSlamMapFrameIdPrefix = "slam_map_";
}  // namespace

std::optional<long long> OccupancyMapNode::parseMapIdFromFrameId(const std::string & frame_id)
{
  const std::string prefix = kSlamMapFrameIdPrefix;
  if (frame_id.rfind(prefix, 0) != 0) {
    return std::nullopt;  // doesn't start with the expected prefix
  }
  const std::string suffix = frame_id.substr(prefix.size());
  if (suffix.empty()) {
    return std::nullopt;
  }
  try {
    size_t consumed = 0;
    long long map_id = std::stoll(suffix, &consumed);
    if (consumed != suffix.size()) {
      return std::nullopt;  // trailing garbage after the number
    }
    return map_id;
  } catch (const std::exception &) {
    return std::nullopt;  // not a valid integer suffix
  }
}

OccupancyMapNode::OccupancyMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("occupancy_map_node", options)
{
  // --- parameters ---
  // Defaults mirror the topic names in ARCHITECTURE.md Section 5.
  // TODO(Section 9): once occupancy_map_cpp constants are added to
  // config/constants.py, these defaults should be kept in sync by hand
  // (there's no shared file between the Python and C++ sides - flag any
  // future rename in both places).
  resolution_ = this->declare_parameter<double>("resolution", 0.10);
  insertion_period_ms_ = this->declare_parameter<int>("insertion_period_ms", 50);
  publish_period_ms_ = this->declare_parameter<int>("publish_period_ms", 1000);
  sync_slop_sec_ = this->declare_parameter<double>("sync_slop_sec", 0.05);
  max_sensor_range_ = this->declare_parameter<double>("max_sensor_range", 3.5);
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "slam_map_0");

  prob_hit_ = this->declare_parameter<double>("prob_hit", 0.95);
  prob_miss_ = this->declare_parameter<double>("prob_miss", 0.45);
  clamping_min_ = this->declare_parameter<double>("clamping_min", 0.012);
  clamping_max_ = this->declare_parameter<double>("clamping_max", 0.97);
  occupancy_thres_ = this->declare_parameter<double>("occupancy_thres", 0.50);

  // Alignment sanity bounds are now evaluated in live_scaler.py (Python),
  // which has access to both the RAW poses and the per-map scale factors
  // needed to compute a geometrically correct metric offset. The parameters
  // are declared there (via constants.MAX_ALIGNMENT_TRANSLATION_M /
  // constants.MAX_ALIGNMENT_GAP_SEC) and the outcome arrives here as a
  // pre-validated MapAlignment message on TOPIC_MAP_ALIGNMENT.

  const std::string pose_topic = this->declare_parameter<std::string>(
    "pose_topic", "/tello_autonomy/current_pose_metric");
  const std::string points_topic = this->declare_parameter<std::string>(
    "points_topic", "/tello_autonomy/current_points_metric");
  const std::string dense_pose_topic = this->declare_parameter<std::string>(
    "dense_pose_topic", "/tello_autonomy/dense_pose_metric");
  const std::string dense_points_topic = this->declare_parameter<std::string>(
    "dense_points_topic", "/tello_autonomy/dense_points_metric");
  const std::string occupancy_topic = this->declare_parameter<std::string>(
    "occupancy_topic", "/tello_autonomy/occupancy_grid");

  const std::string alignment_topic = this->declare_parameter<std::string>(
    "alignment_topic", "/tello_autonomy/map_alignment");

  octree_ = std::make_unique<octomap::OcTree>(resolution_);
  octree_->setProbHit(prob_hit_);
  octree_->setProbMiss(prob_miss_);
  octree_->setClampingThresMin(clamping_min_);
  octree_->setClampingThresMax(clamping_max_);
  octree_->setOccupancyThres(occupancy_thres_);
  // regular_stream_map_id_ starts as nullopt - the first regular-stream
  // job's map_id is adopted without triggering a re-init event.
  regular_stream_map_id_ = std::nullopt;

  // --- publisher ---
  occupancy_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    occupancy_topic, rclcpp::QoS(10));

  // --- TOPIC_MAP_ALIGNMENT subscriber ---
  // live_scaler.py publishes here whenever it detects a map_id transition on
  // the raw pose stream. This callback either arms pending_alignment_ (accepted
  // case) or hard-resets the OcTree (rejected case), both under octree_mutex_.
  alignment_sub_ = this->create_subscription<tello_autonomy_msgs::msg::MapAlignment>(
    alignment_topic, rclcpp::QoS(10),
    std::bind(&OccupancyMapNode::mapAlignmentCallback, this, std::placeholders::_1));

  // --- synchronized subscribers ---
  // Pose and points are published as two separate topics per frame by
  // live_scaler.py. insertPointCloud() needs both as one unit (an origin
  // + a cloud), so pair them here by timestamp before anything reaches
  // the queue. See the class-level comment in the header for why this
  // exists - it's a gap ARCHITECTURE.md Section 12.2 didn't spell out.
  pose_sub_.subscribe(this, pose_topic, rmw_qos_profile_sensor_data);
  points_sub_.subscribe(this, points_topic, rmw_qos_profile_sensor_data);

  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), pose_sub_, points_sub_);
  synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
  synchronizer_->registerCallback(
    std::bind(
      &OccupancyMapNode::syncedCallback, this,
      std::placeholders::_1, std::placeholders::_2, /*is_dense=*/false));

  // Also subscribe to dense metric points published by ScaleFactorManager
  dense_pose_sub_.subscribe(this, dense_pose_topic, rmw_qos_profile_sensor_data);
  dense_points_sub_.subscribe(this, dense_points_topic, rmw_qos_profile_sensor_data);

  dense_synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), dense_pose_sub_, dense_points_sub_);
  dense_synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
  dense_synchronizer_->registerCallback(
    std::bind(
      &OccupancyMapNode::syncedCallback, this,
      std::placeholders::_1, std::placeholders::_2, /*is_dense=*/true));

  // --- decoupled publish timer (Section 12.2: "two independently
  // tunable rates, not one") ---
  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&OccupancyMapNode::publishTimerCallback, this));

  // --- worker thread: the only thing that ever calls insertPointCloud ---
  worker_thread_ = std::thread(&OccupancyMapNode::workerLoop, this);

  RCLCPP_INFO(
    this->get_logger(),
    "occupancy_map_node started: resolution=%.3fm insertion_period=%dms "
    "publish_period=%dms sync_slop=%.3fs. OcTree will reset automatically "
    "whenever the incoming map_id (parsed from pose/points frame_id) changes.",
    resolution_, insertion_period_ms_, publish_period_ms_, sync_slop_sec_);
}

OccupancyMapNode::~OccupancyMapNode()
{
  shutting_down_.store(true);
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void OccupancyMapNode::syncedCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points,
  bool is_dense_stream)
{
  // Deliberately the only real work this callback does beyond parsing
  // the map_id: push a job onto the queue and return. All insertion (and
  // any resulting octree reset) happens on worker_thread_ - see
  // workerLoop(). Parsing here is cheap (a prefix check + stoll) and
  // avoids re-deriving it later; it does NOT touch octree_ or
  // regular_stream_map_id_, so it needs no lock.
  const auto map_id = parseMapIdFromFrameId(pose->header.frame_id);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    job_queue_.push_back(InsertionJob{pose, points, map_id, is_dense_stream});
  }
  queue_cv_.notify_one();
}

void OccupancyMapNode::workerLoop()
{
  const auto period = std::chrono::milliseconds(insertion_period_ms_);

  while (!shutting_down_.load()) {
    std::deque<InsertionJob> jobs_to_process;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      // Wake early if work arrives, otherwise wake at most every
      // insertion_period_ms_ to re-check the shutdown flag.
      queue_cv_.wait_for(
        lock, period,
        [this] {return !job_queue_.empty() || shutting_down_.load();});
      jobs_to_process.swap(job_queue_);
    }

    if (jobs_to_process.empty()) {
      continue;
    }

    for (const auto & job : jobs_to_process) {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(*job.points, pcl_cloud);

      octomap::Pointcloud octomap_cloud;
      octomap_cloud.reserve(pcl_cloud.size());
      for (const auto & pt : pcl_cloud.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
          octomap_cloud.push_back(pt.x, pt.y, pt.z);
        }
      }

      const octomap::point3d origin(
        static_cast<float>(job.pose->pose.position.x),
        static_cast<float>(job.pose->pose.position.y),
        static_cast<float>(job.pose->pose.position.z));

      // Track the most recent pose seen under the CURRENT map_id, so that
      // if the next job's map_id changes, we have an anchor to align
      // against. Must happen before the map_id comparison below uses it.
      Eigen::Vector3d incoming_pos(origin.x(), origin.y(), origin.z());
      Eigen::Quaterniond incoming_rot(
        job.pose->pose.orientation.w, job.pose->pose.orientation.x,
        job.pose->pose.orientation.y, job.pose->pose.orientation.z);

      std::lock_guard<std::mutex> lock(octree_mutex_);

      // Alignment gate: applies to BOTH regular and dense jobs. A map_id
      // > 0 that hasn't had its MapAlignment message processed yet must
      // not have ANY points inserted - dense (backprojected) points are
      // just as capable of landing in the wrong local frame as regular
      // SLAM points, so exempting them from this gate (as before) let
      // mis-registered dense points slip into the OcTree during every
      // re-init, independently of the Python-side fix in
      // scale_factor_manager.py.
      if (job.map_id.has_value() && *job.map_id > 0 &&
        map_ids_already_aligned_.count(*job.map_id) == 0)
      {
        RCLCPP_DEBUG(
          this->get_logger(),
          "map_id %lld (dense=%d): job before alignment message - skipping frame until aligned.",
          *job.map_id, static_cast<int>(job.is_dense_stream));
        continue;
      }

      // Detection: regular stream only. Dense jobs must never advance
      // regular_stream_map_id_ - see the class-level header comment on
      // why the dense stream's map_id can legitimately race the regular
      // stream's.
      if (!job.is_dense_stream && job.map_id.has_value()) {
        regular_stream_map_id_ = job.map_id;
        map_frame_id_ = "slam_map_" + std::to_string(*job.map_id);
      }

      if (max_sensor_range_ > 0.0) {
        octree_->insertPointCloud(octomap_cloud, origin, max_sensor_range_);
      } else {
        octree_->insertPointCloud(octomap_cloud, origin);
      }
    }
  }
}

void OccupancyMapNode::mapAlignmentCallback(
  const tello_autonomy_msgs::msg::MapAlignment::SharedPtr msg)
{
  // Called on the ROS2 executor thread whenever live_scaler.py publishes
  // a MapAlignment. Runs under octree_mutex_ so any modification to
  // octree_ is visible to workerLoop() atomically.
  //
  // map_ids_already_aligned_ is checked here as a second-guard against
  // duplicate application (e.g. if the message is delivered twice, or if
  // workerLoop() already processed this transition before the message arrived).
  std::lock_guard<std::mutex> lock(octree_mutex_);
  const long long new_id = msg->new_map_id;

  if (map_ids_already_aligned_.count(new_id) != 0) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "mapAlignmentCallback: map_id %lld already processed - ignoring duplicate.",
      new_id);
    return;
  }
  map_ids_already_aligned_.insert(new_id);

  if (msg->accepted) {
    // live_scaler.py pre-aligns BOTH streams (regular and dense) before they
    // reach this node, so there is nothing to transform here. This callback's
    // job is purely to gate workerLoop() via map_ids_already_aligned_ and to
    // hard-reset the OcTree when a rejection is received.
    RCLCPP_WARN(
      this->get_logger(),
      "Received accepted alignment for map %lld -> %lld (offset %.3fm over %.2fs). "
      "Points pre-aligned by live_scaler.py - gate opened for map_id %lld.",
      static_cast<long long>(msg->old_map_id), static_cast<long long>(new_id),
      msg->offset_m, msg->gap_sec, static_cast<long long>(new_id));
  } else {
    octree_ = std::make_unique<octomap::OcTree>(resolution_);
    octree_->setProbHit(prob_hit_);
    octree_->setProbMiss(prob_miss_);
    octree_->setClampingThresMin(clamping_min_);
    octree_->setClampingThresMax(clamping_max_);
    octree_->setOccupancyThres(occupancy_thres_);
    RCLCPP_WARN(
      this->get_logger(),
      "Rejected alignment for map %lld -> %lld (offset %.3fm / gap %.2fs) - "
      "OcTree hard-reset. New map starts with empty geometry.",
      static_cast<long long>(msg->old_map_id), static_cast<long long>(new_id),
      msg->offset_m, msg->gap_sec);
  }
  force_publish_next_tick_ = true;
}

void OccupancyMapNode::publishTimerCallback()
{
  octomap_msgs::msg::Octomap msg;
  std::string frame_id_snapshot;

  {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    if (octree_->size() == 0 && !force_publish_next_tick_) {
      return;  // nothing inserted yet, skip this publish tick
    }
    force_publish_next_tick_ = false;
    if (!octomap_msgs::fullMapToMsg(*octree_, msg)) {
      RCLCPP_WARN(this->get_logger(), "fullMapToMsg failed, skipping this publish tick");
      return;
    }
    // Read map_frame_id_ under the same lock that protects writes to it
    // in workerLoop(), rather than after releasing the lock.
    frame_id_snapshot = map_frame_id_;
  }

  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_snapshot;
  occupancy_pub_->publish(msg);
}

}  // namespace occupancy_map_cpp
