#ifndef OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_
#define OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tello_autonomy_msgs/msg/map_alignment.hpp>

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
  // True for jobs from the dense_synchronizer_ (dense_pose_metric /
  // dense_points_metric published by scale_factor_manager.py). These
  // jobs participate in INSERTION but must never trigger re-init
  // detection - see the split design in OccupancyMapNode.
  bool is_dense_stream = false;
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
// Map re-init handling (split detection/insertion design):
//
//   Two synchronizers feed the same job_queue_: synchronizer_ (the
//   regular per-frame SLAM stream, current_pose_metric /
//   current_points_metric) and dense_synchronizer_ (the periodic
//   backprojection from scale_factor_manager.py, dense_pose_metric /
//   dense_points_metric). Both contribute points to the OcTree, but
//   only the regular stream is allowed to drive re-init detection.
//
//   Points from BOTH streams are pre-aligned into the global Map 0
//   coordinate frame by live_scaler.py (regular stream) and
//   scale_factor_manager.py (dense stream) before they reach this node.
//   This node does NOT apply any additional transform at insertion time.
//   The mapAlignmentCallback() callback's only remaining job is to gate
//   workerLoop() via map_ids_already_aligned_ (holding back any job for
//   a not-yet-registered map_id) and to hard-reset the OcTree when
//   live_scaler.py signals a rejected alignment.
class OccupancyMapNode : public rclcpp::Node
{
public:
  explicit OccupancyMapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OccupancyMapNode() override;

private:
  // Fired by the message_filters synchronizer once a pose/points pair
  // within sync_slop_sec_ of each other has been found. Must stay cheap.
  // is_dense_stream tags which synchronizer produced this pair so
  // workerLoop() can gate re-init detection to the regular stream only.
  void syncedCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points,
    bool is_dense_stream);

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

  // Called on the ROS2 executor thread when perception/live_scaler.py
  // publishes a MapAlignment message. Sets pending_alignment_ (accepted)
  // or hard-resets the OcTree (rejected) under octree_mutex_.
  void mapAlignmentCallback(
    const tello_autonomy_msgs::msg::MapAlignment::SharedPtr msg);

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

  // --- Map re-init detection: ONLY driven by the regular stream ---
  // (current_pose_metric / current_points_metric). The dense stream
  // (dense_pose_metric / dense_points_metric, from
  // scale_factor_manager.py's periodic backprojection) shares the same
  // job_queue_ and worker thread for INSERTION, but must never be
  // allowed to trigger a re-init decision itself - it's not a continuous
  // per-frame signal, so its map_id can legitimately arrive slightly
  // out of order relative to the regular stream even when nothing about
  // SLAM's actual tracking state changed.
  //
  // regular_stream_map_id_: the last map_id seen from the regular
  // stream. std::nullopt = no regular-stream job yet.
  std::optional<long long> regular_stream_map_id_;

  // map_ids_already_aligned_: set of map_ids for which we have already
  // computed and committed an alignment (or hard reset). ORB-SLAM3 map
  // ids are monotonically increasing - a transition to an already-seen
  // id is definitionally cross-topic noise, not a real re-init.
  std::set<long long> map_ids_already_aligned_;

  // --- Map re-init alignment ---
  // live_scaler.py pre-aligns all points into the global Map 0 frame
  // before publishing them. This node does NOT apply any additional
  // transform at insertion time. mapAlignmentCallback() is purely a
  // gate (map_ids_already_aligned_) and a hard-reset trigger (rejected
  // alignment -> fresh OcTree).
  bool force_publish_next_tick_ = false;

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

  // Subscriber to TOPIC_MAP_ALIGNMENT published by live_scaler.py.
  // Fires on the ROS2 executor thread; sets/clears pending_alignment_ under
  // octree_mutex_ so workerLoop() sees a consistent transform at insertion time.
  rclcpp::Subscription<tello_autonomy_msgs::msg::MapAlignment>::SharedPtr alignment_sub_;
};

}  // namespace occupancy_map_cpp

#endif  // OCCUPANCY_MAP_CPP__OCCUPANCY_MAP_NODE_HPP_
