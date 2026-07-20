#ifndef OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_
#define OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace occupancy_map_cpp
{

// One unit of work for the worker thread: a pose paired with the point
// cloud captured from that pose. Built by the message_filters
// synchronizer so insertPointCloud() always gets a matched pair.
struct InsertionJob
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr points;
};

// Design note (see ARCHITECTURE.md Section 12.2 and the chat discussion
// that scaffolded this file):
//
//   - The ROS callback (syncedCallback) does nothing but push a job onto
//     a queue and return. This mirrors the fix from scale_factor_manager.py
//     Bug #3/#4: expensive work never runs on a subscription callback.
//   - worker_thread_ drains that queue on its own schedule
//     (insertion_period_ms_) and does the actual OcTree::insertPointCloud
//     ray-casting.
//   - publish_timer_ runs independently, at a much lower, decoupled rate
//     (publish_period_ms_), serializing the current octree state.
//   - octree_mutex_ is the only thing shared between the worker thread
//     and the publish timer; the queue mutex is separate and only ever
//     touches job_queue_.
class OccupancyMapNode : public rclcpp::Node
{
public:
  explicit OccupancyMapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OccupancyMapNode() override;

private:
  // Fired by the message_filters synchronizer once a pose/points pair
  // within sync_slop_sec_ of each other has been found. Must stay cheap.
  void syncedCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points);

  // Runs on publish_timer_'s own thread via the executor; serializes and
  // publishes the current octree. Independent of insertion_period_ms_.
  void publishTimerCallback();

  // Runs on worker_thread_ for the lifetime of the node. Drains
  // job_queue_ and calls OcTree::insertPointCloud for each job.
  void workerLoop();

  // --- parameters (loaded once in the constructor) ---
  double resolution_;
  int insertion_period_ms_;
  int publish_period_ms_;
  double sync_slop_sec_;
  double max_sensor_range_;  // <= 0 means unlimited, passed to insertPointCloud
  std::string map_frame_id_;

  // --- the map itself ---
  std::unique_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;

  // --- queue shared between syncedCallback (producer) and workerLoop (consumer) ---
  std::deque<InsertionJob> job_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::atomic<bool> shutting_down_{false};
  std::thread worker_thread_;

  // --- ROS interfaces ---
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> points_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::PoseStamped, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr occupancy_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace occupancy_map_cpp

#endif  // OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_
