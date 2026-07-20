#include "occupancy_map_cpp/occupancy_map_node.hpp"

#include <chrono>
#include <cmath>

#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace occupancy_map_cpp
{

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
  max_sensor_range_ = this->declare_parameter<double>("max_sensor_range", -1.0);
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");

  const std::string pose_topic = this->declare_parameter<std::string>(
    "pose_topic", "/tello_autonomy/current_pose_metric");
  const std::string points_topic = this->declare_parameter<std::string>(
    "points_topic", "/tello_autonomy/current_points_metric");
  const std::string occupancy_topic = this->declare_parameter<std::string>(
    "occupancy_topic", "/tello_autonomy/occupancy_grid");

  octree_ = std::make_unique<octomap::OcTree>(resolution_);

  // --- publisher ---
  occupancy_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    occupancy_topic, rclcpp::QoS(10));

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
      std::placeholders::_1, std::placeholders::_2));

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
    "publish_period=%dms sync_slop=%.3fs",
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
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points)
{
  // Deliberately the only thing this callback does. All real work
  // happens on worker_thread_ - see workerLoop().
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    job_queue_.push_back(InsertionJob{pose, points});
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

      std::lock_guard<std::mutex> lock(octree_mutex_);
      if (max_sensor_range_ > 0.0) {
        octree_->insertPointCloud(octomap_cloud, origin, max_sensor_range_);
      } else {
        octree_->insertPointCloud(octomap_cloud, origin);
      }
    }
  }
}

void OccupancyMapNode::publishTimerCallback()
{
  octomap_msgs::msg::Octomap msg;

  {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    if (octree_->size() == 0) {
      return;  // nothing inserted yet, skip this publish tick
    }
    if (!octomap_msgs::fullMapToMsg(*octree_, msg)) {
      RCLCPP_WARN(this->get_logger(), "fullMapToMsg failed, skipping this publish tick");
      return;
    }
  }

  msg.header.stamp = this->now();
  msg.header.frame_id = map_frame_id_;
  occupancy_pub_->publish(msg);
}

}  // namespace occupancy_map_cpp
