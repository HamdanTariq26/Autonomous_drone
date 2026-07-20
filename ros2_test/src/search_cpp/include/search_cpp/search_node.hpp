#ifndef SEARCH_CPP__SEARCH_NODE_HPP_
#define SEARCH_CPP__SEARCH_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include <octomap_manager_shim/octomap_manager_shim.hpp>
#include <tello_autonomy_msgs/srv/search_plan.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace search_cpp
{

// Drone physical bounding box (meters), padded by a small safety margin.
// Tello body: 98 x 92.5 x 41 mm  →  0.098 x 0.0925 x 0.041 m
// We add a 10 mm margin on every axis for sensor tolerances and wind drift.
static constexpr double DRONE_BB_X = 0.108;  // 98mm + 10mm margin
static constexpr double DRONE_BB_Y = 0.103;  // 92.5mm + 10mm margin
static constexpr double DRONE_BB_Z = 0.051;  // 41mm + 10mm margin

// BIT* planning budget: stop after this many seconds even if not optimal.
// 0.5 s gives BIT* enough iterations for room-scale environments while
// keeping service latency acceptable in a receding-horizon loop.
static constexpr double PLAN_TIMEOUT_SEC = 0.5;

class SearchNode : public rclcpp::Node
{
public:
  explicit SearchNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SearchNode() = default;

private:
  // ---- ROS2 interface ----
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Service<tello_autonomy_msgs::srv::SearchPlan>::SharedPtr plan_srv_;

  // ---- OctoMap/shim ----
  std::shared_ptr<const octomap::OcTree> octree_;
  std::shared_ptr<octomap_manager_shim::OctomapManagerShim> shim_;
  mutable std::mutex map_mutex_;  // guards octree_ and shim_ from concurrent map update

  // ---- parameters ----
  double plan_timeout_sec_;
  Eigen::Vector3d drone_bbox_;

  // ---- callbacks ----
  void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  void planCallback(
    const std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan::Request> req,
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan::Response> res);

  // ---- OMPL helpers ----
  bool isStateValid(
    const ob::State * state,
    const std::shared_ptr<const octomap_manager_shim::OctomapManagerShim> & shim_snap,
    bool allow_unknown = false) const;

  // Endpoint-only check: accepts unknown (unscanned) but rejects known-occupied.
  // Used for start and goal states only — interior path states use isStateValid().
  bool isEndpointAcceptable(
    const Eigen::Vector3d & pos,
    const std::shared_ptr<const octomap_manager_shim::OctomapManagerShim> & shim) const;

  std::vector<geometry_msgs::msg::PoseStamped> omplPathToMsg(
    const og::PathGeometric & path,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;
};

}  // namespace search_cpp

#endif  // SEARCH_CPP__SEARCH_NODE_HPP_
