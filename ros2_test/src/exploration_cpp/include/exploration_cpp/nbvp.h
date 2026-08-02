/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Ported to ROS2 for tello_autonomy (ARCHITECTURE.md Section 12.4).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 //Changed
#ifndef NBVP_H_
#define NBVP_H_

#include <vector>
#include <fstream>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <tello_autonomy_msgs/msg/segment.hpp>

#include <octomap_manager_shim/octomap_manager_shim.hpp>
#include <tello_autonomy_msgs/srv/nbv_plan.hpp>
#include <exploration_cpp/mesh_structure.h>
#include <exploration_cpp/tree.hpp>
#include <exploration_cpp/rrt.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

template<typename stateVec>
class nbvPlanner : public rclcpp::Node
{
  // No nh_/nh_private_ members - ROS2 has no NodeHandle concept.
  // `this` (the node itself) is passed to create_subscription/
  // create_publisher/create_service calls in the constructor.

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posClient_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomClient_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr peerPosClient1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr peerPosClient2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr peerPosClient3_;
  rclcpp::Subscription<tello_autonomy_msgs::msg::Segment>::SharedPtr evadeClient_;
  rclcpp::Publisher<tello_autonomy_msgs::msg::Segment>::SharedPtr evadePub_;
  rclcpp::Service<tello_autonomy_msgs::srv::NbvPlan>::SharedPtr plannerService_;

  // NEW: replaces the three deleted pointcloud_sub_* subscribers
  // (pointcloud_sub_, pointcloud_sub_cam_up_, pointcloud_sub_cam_down_).
  // occupancy_map_cpp already owns point-cloud insertion (Section 12.2) and
  // publishes the finished octree here - this node only ever reads it.
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr occupancyMapClient_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr slamPointsClient_;

  Params params_;
  mesh::StlMesh * mesh_;
  octomap_manager_shim::OctomapManagerShim * manager_;
  std::mutex map_mutex_;

  bool ready_;

 public:
  typedef std::vector<stateVec> vector_t;
  TreeBase<stateVec> * tree_;

  explicit nbvPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~nbvPlanner();
  bool setParams();
  void posCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr pose);
  void occupancyMapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void slamPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void plannerCallback(
    const std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan::Request> req,
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan::Response> res);
  void evasionCallback(const tello_autonomy_msgs::msg::Segment::SharedPtr segmentMsg);
};
}  // namespace nbvInspection

#endif  // NBVP_H_