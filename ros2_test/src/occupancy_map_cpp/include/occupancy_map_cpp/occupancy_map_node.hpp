#ifndef OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_
#define OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
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
//
// map_id is parsed once, cheaply, in syncedCallback() (see
// parseMapIdFromFrameId()) and carried through to the worker thread so
// the re-init detection/reset logic below can run under the same
// octree_mutex_ that guards insertion - never in the callback itself,
// which must stay cheap and must never touch octree_.
struct InsertionJob
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr points;
  std::optional<long long> map_id;
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
//
// Map re-init handling (pipeline_audit.md, Finding #1):
//   Every pose/points message's header.frame_id already carries the
//   producing map_id as "slam_map_<N>" (see
//   config.constants.SLAM_MAP_FRAME_ID_PREFIX on the Python side, and
//   PublishCurrentPoseAndPoints()/PublishLiveMapData() in common.cpp,
//   which is what actually stamps it there). Previously this node never
//   looked at that field at all - it inserted every point it ever
//   received into one never-reset OcTree regardless of which map
//   produced it. When SLAM loses tracking and re-initializes, the new
//   map_id's origin is wherever the drone physically was at that
//   moment, NOT the same physical origin as the old map - so blending
//   old- and new-map points into one octree produces duplicated/offset
//   geometry that exploration_cpp's RRT then trusts completely.
//
//   Fix: track current_map_id_ (guarded by octree_mutex_, since its
//   lifetime is tied to octree_'s). Whenever a job's map_id differs from
//   current_map_id_, clear octree_ before inserting that job's points,
//   and update map_frame_id_ so publishTimerCallback() stamps outgoing
//   octomap messages with whichever map is actually live right now
//   (fixes the frame_id going stale after a re-init, too). This is
//   option (a) from the audit - reset-on-reinit - not an attempt to
//   align old and new maps via a relative transform; a partially
//   remapped room beats a corrupted one, and this is by far the
//   cheaper, lower-risk fix of the two options discussed.
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

  // Parses the map_id encoded in a header.frame_id of the form
  // "slam_map_<N>" (SLAM_MAP_FRAME_ID_PREFIX + map_id on the Python
  // side). Returns std::nullopt if frame_id doesn't match that pattern
  // (treated as "unknown map" - never crashes, never resets on a
  // malformed/unexpected frame_id, only on a genuine, parseable change).
  static std::optional<long long> parseMapIdFromFrameId(const std::string & frame_id);

  // --- parameters (loaded once in the constructor) ---
  double resolution_;
  int insertion_period_ms_;
  int publish_period_ms_;
  double sync_slop_sec_;
  double max_sensor_range_;  // <= 0 means unlimited, passed to insertPointCloud
  std::string map_frame_id_;

  // Cached OcTree construction parameters, needed to rebuild a fresh
  // OcTree with the same settings on a map re-init (see workerLoop()).
  double prob_hit_;
  double prob_miss_;
  double clamping_min_;
  double clamping_max_;
  double occupancy_thres_;

  // --- the map itself ---
  std::unique_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;

  // Which map_id's points are currently in octree_. std::nullopt means
  // "no points inserted yet" - the very first job's map_id is adopted
  // without triggering a reset (there is nothing to reset).
  std::optional<long long> current_map_id_;

  // --- queue shared between syncedCallback (producer) and workerLoop (consumer) ---
  std::deque<InsertionJob> job_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::atomic<bool> shutting_down_{false};
  std::thread worker_thread_;

  // --- ROS interfaces ---
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> points_sub_;

  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> dense_pose_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> dense_points_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::PoseStamped, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> dense_synchronizer_;

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr occupancy_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace occupancy_map_cpp

#endif  // OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_
