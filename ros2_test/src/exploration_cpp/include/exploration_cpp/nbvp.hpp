/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Ported to ROS2 for tello_autonomy (ARCHITECTURE.md Section 12.4/13.3).
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

#ifndef NBVP_HPP_
#define NBVP_HPP_

#include <fstream>
#include <limits>
#include <memory>
#include <Eigen/Dense>

#include <visualization_msgs/msg/marker.hpp>
#include <octomap_msgs/conversions.h>

#include <exploration_cpp/nbvp.h>

// Convenience macro to get the absolute yaw difference
#define ANGABS(x) (fmod(fabs(x),2.0*M_PI)<M_PI?fmod(fabs(x),2.0*M_PI):2.0*M_PI-fmod(fabs(x),2.0*M_PI))

using namespace Eigen;
using namespace std::placeholders;

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::nbvPlanner(const rclcpp::NodeOptions & options)
    : rclcpp::Node("nbv_planner_node", options)
{
  // No nh_/nh_private_ to store - `this` is the node.

  // The shim owns no ROS wiring itself (Section 12.3's design: it's a
  // plain reader over whatever octree setOctree() gives it). It starts
  // with no tree until the first occupancyMapCallback() fires.
  manager_ = new octomap_manager_shim::OctomapManagerShim();
  manager_->setTreatUnknownAsOccupied(false);

  // Set up the topics and services
  params_.inspectionPath_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "inspectionPath", 1000);
  evadePub_ = this->create_publisher<tello_autonomy_msgs::msg::Segment>(
    "/evasionSegment", 100);
  plannerService_ = this->create_service<tello_autonomy_msgs::srv::NbvPlan>(
    "nbvplanner",
    std::bind(&nbvInspection::nbvPlanner<stateVec>::plannerCallback, this, _1, _2));
  posClient_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose", 10, std::bind(&nbvInspection::nbvPlanner<stateVec>::posCallback, this, _1));
  odomClient_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry", 10, std::bind(&nbvInspection::nbvPlanner<stateVec>::odomCallback, this, _1));

  // NEW (replaces the three deleted pointcloud_sub_* subscriptions -
  // Section 13.3, Open Issue #11). occupancy_map_cpp already owns
  // insertion; this node only ever reads the finished octree.
  occupancyMapClient_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/tello_autonomy/occupancy_grid", 10,
    std::bind(&nbvInspection::nbvPlanner<stateVec>::occupancyMapCallback, this, _1));

  slamPointsClient_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/tello_autonomy/current_points_metric", 10,
    std::bind(&nbvInspection::nbvPlanner<stateVec>::slamPointsCallback, this, _1));

  if (!setParams()) {
    RCLCPP_ERROR(this->get_logger(), "Could not start the planner. Parameters missing!");
  }

  // Precompute the camera field of view boundaries. Unchanged - pure
  // Eigen math, no ROS1 coupling.
  for (size_t i = 0; i < params_.camPitch_.size(); i++) {
    double pitch = M_PI * params_.camPitch_[i] / 180.0;
    double camTop = (pitch - M_PI * params_.camVertical_[i] / 360.0) + M_PI / 2.0;
    double camBottom = (pitch + M_PI * params_.camVertical_[i] / 360.0) - M_PI / 2.0;
    double side = M_PI * (params_.camHorizontal_[i]) / 360.0 - M_PI / 2.0;
    Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
    Vector3d top(cos(camTop), 0.0, -sin(camTop));
    Vector3d right(cos(side), sin(side), 0.0);
    Vector3d left(cos(side), -sin(side), 0.0);
    AngleAxisd m = AngleAxisd(pitch, Vector3d::UnitY());
    Vector3d rightR = m * right;
    Vector3d leftR = m * left;
    rightR.normalize();
    leftR.normalize();
    std::vector<Eigen::Vector3d> camBoundNormals;
    camBoundNormals.push_back(bottom);
    camBoundNormals.push_back(top);
    camBoundNormals.push_back(rightR);
    camBoundNormals.push_back(leftR);
    params_.camBoundNormals_.push_back(camBoundNormals);
  }

  // Load mesh from STL file if provided (Open Issue #7: kept, gated).
  // ROS2 parameters are already scoped to this node, so the manual
  // "ns + /param_name" concatenation from ROS1 is dropped - just declare
  // the plain name.
  std::string stlPath = this->declare_parameter<std::string>("stl_file_path", "");
  mesh_ = nullptr;
  if (stlPath.length() > 0) {
    params_.meshResolution_ = this->declare_parameter<double>("mesh_resolution", 0.1);
    std::fstream stlFile;
    stlFile.open(stlPath.c_str());
    if (stlFile.is_open()) {
      mesh_ = new mesh::StlMesh(stlFile);
      mesh_->setResolution(params_.meshResolution_);
      // NOTE: StlMesh::setOctomapManager expects the manager type it was
      // written against. Since manager_ is now OctomapManagerShim*, this
      // call site (and mesh_structure.h/.cpp's signature) needs the same
      // type swap applied here - not yet done in this file; flag before
      // enabling mesh_.
      mesh_->setOctomapManager(manager_);
      mesh_->setCameraParams(
        params_.camPitch_, params_.camHorizontal_, params_.camVertical_, params_.gainRange_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unable to open STL file");
    }
  }

  // Initialize the tree instance.
  tree_ = new RrtTree(mesh_, manager_, this);
  tree_->setParams(params_);
  peerPosClient1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "peer_pose_1", 10,
    std::bind(&nbvInspection::RrtTree::setPeerStateFromPoseMsg1, tree_, _1));
  peerPosClient2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "peer_pose_2", 10,
    std::bind(&nbvInspection::RrtTree::setPeerStateFromPoseMsg2, tree_, _1));
  peerPosClient3_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "peer_pose_3", 10,
    std::bind(&nbvInspection::RrtTree::setPeerStateFromPoseMsg3, tree_, _1));
  // Collaborative collision avoidance (don't hit your peer) - multi-agent,
  // gated per Open Issue #7, never wired to anything unless peer topics
  // are actually published.
  evadeClient_ = this->create_subscription<tello_autonomy_msgs::msg::Segment>(
    "/evasionSegment", 10, std::bind(&nbvInspection::TreeBase<stateVec>::evade, tree_, _1));

  // Not yet ready. Needs a position message first.
  ready_ = false;
}

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::~nbvPlanner()
{
  if (manager_) {
    delete manager_;
  }
  if (mesh_) {
    delete mesh_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::posCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
{
  tree_->setStateFromPoseMsg(*pose);
  ready_ = true;
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr pose)
{
  tree_->setStateFromOdometryMsg(*pose);
  ready_ = true;
}

// NEW method - has no ROS1 equivalent. Replaces the three deleted
// insertPointcloudWithTf* callbacks (Section 13.3). Decodes the finished
// octree occupancy_map_cpp publishes and hands it to the shim; does NOT
// do any insertion itself.
template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::occupancyMapCallback(
    const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  octomap::AbstractOcTree * abstract_tree = octomap_msgs::fullMsgToMap(*msg);
  if (!abstract_tree) {
    RCLCPP_WARN(this->get_logger(), "occupancyMapCallback: fullMsgToMap failed, dropping message");
    return;
  }
  std::shared_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree *>(abstract_tree));
  if (!tree) {
    RCLCPP_WARN(this->get_logger(), "occupancyMapCallback: decoded map was not an OcTree");
    return;
  }
  manager_->setOctree(tree);
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::slamPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (tree_) {
    ((RrtTree*)tree_)->setLiveSlamPoints(msg);
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::plannerCallback(
    const std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan::Request> req,
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan::Response> res)
{
  rclcpp::Time computationTime = this->now();
  // Check that planner is ready to compute path.
  if (!rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Exploration finished. Not planning any further moves.");
    return;
  }

  if (!ready_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Planner not set up: Planner not ready!");
    return;
  }
  if (manager_ == nullptr) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Planner not set up: No octomap available!");
    return;
  }
  if (manager_->getMapSize().norm() <= 0.0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Planner not set up: Octomap is empty!");
    return;
  }
  res->path.clear();

  // Clear old tree and reinitialize.
  tree_->clear();
  tree_->initialize();
  vector_t path;
  // Iterate the tree construction method.
  int loopCount = 0;
  while ((!tree_->gainFound() || tree_->getCounter() < params_.initIterations_) && rclcpp::ok()) {
    if (tree_->getCounter() > params_.cuttoffIterations_) {
      RCLCPP_INFO(this->get_logger(), "No gain found, returning to previous point");
      res->path = tree_->getPathBackToPrevious(req->header.frame_id);
      return;
    }
    if (loopCount > 1000 * (tree_->getCounter() + 1)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Exceeding maximum failed iterations, return to previous point!");
      res->path = tree_->getPathBackToPrevious(req->header.frame_id);
      return;
    }
    tree_->iterate(1);
    loopCount++;
  }
  // Extract the best edge.
  res->path = tree_->getBestEdge(req->header.frame_id);

  tree_->memorizeBestBranch();
  // Publish path to block for other agents (multi agent only).
  tello_autonomy_msgs::msg::Segment segment;
  segment.header.stamp = this->now();
  segment.header.frame_id = params_.navigationFrame_;
  if (!res->path.empty()) {
    segment.poses.push_back(res->path.front());
    segment.poses.push_back(res->path.back());
  }
  evadePub_->publish(segment);
  RCLCPP_INFO(
    this->get_logger(), "Path computation lasted %2.3fs",
    (this->now() - computationTime).seconds());
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::setParams()
{
  bool ret = true;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  params_.v_max_ = this->declare_parameter<double>("system.v_max", 0.25);
  params_.dyaw_max_ = this->declare_parameter<double>("system.dyaw_max", 0.5);
  params_.camPitch_ = this->declare_parameter<std::vector<double>>(
    "system.camera.pitch", std::vector<double>{15.0});
  params_.camHorizontal_ = this->declare_parameter<std::vector<double>>(
    "system.camera.horizontal", std::vector<double>{90.0});
  params_.camVertical_ = this->declare_parameter<std::vector<double>>(
    "system.camera.vertical", std::vector<double>{60.0});

  if (params_.camPitch_.size() != params_.camHorizontal_.size() ||
    params_.camPitch_.size() != params_.camVertical_.size())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Specified camera fields of view unclear: parameter vector length mismatch! "
      "Setting to default.");
    params_.camPitch_ = {15.0};
    params_.camHorizontal_ = {90.0};
    params_.camVertical_ = {60.0};
  }
  params_.igFree_ = this->declare_parameter<double>("nbvp.gain.free", 0.0);
  params_.igOccupied_ = this->declare_parameter<double>("nbvp.gain.occupied", 0.0);
  params_.igUnmapped_ = this->declare_parameter<double>("nbvp.gain.unmapped", 1.0);
  params_.igArea_ = this->declare_parameter<double>("nbvp.gain.area", 1.0);
  params_.degressiveCoeff_ = this->declare_parameter<double>("nbvp.gain.degressive_coeff", 0.25);
  params_.yawPenalty_ = this->declare_parameter<double>("nbvp.gain.yaw_penalty", 0.5);
  params_.extensionRange_ = this->declare_parameter<double>("nbvp.tree.extension_range", 1.0);
  params_.initIterations_ = this->declare_parameter<int>("nbvp.tree.initial_iterations", 150);
  params_.dt_ = this->declare_parameter<double>("nbvp.dt", 0.1);
  params_.gainRange_ = this->declare_parameter<double>("nbvp.gain.range", 1.0);

  // These six were REQUIRED in the original (ret=false if missing, no
  // default). ROS2's declare_parameter needs a default to avoid throwing,
  // so NaN is used as a "not actually set" sentinel and checked below -
  // preserves the original "warn + fail setParams()" behavior rather than
  // silently accepting NaN bounds.
  params_.minX_ = this->declare_parameter<double>("bbx.minX", nan);
  if (std::isnan(params_.minX_)) {
    RCLCPP_WARN(this->get_logger(), "No x-min value specified. Looking for bbx.minX");
    ret = false;
  }
  params_.minY_ = this->declare_parameter<double>("bbx.minY", nan);
  if (std::isnan(params_.minY_)) {
    RCLCPP_WARN(this->get_logger(), "No y-min value specified. Looking for bbx.minY");
    ret = false;
  }
  params_.minZ_ = this->declare_parameter<double>("bbx.minZ", nan);
  if (std::isnan(params_.minZ_)) {
    RCLCPP_WARN(this->get_logger(), "No z-min value specified. Looking for bbx.minZ");
    ret = false;
  }
  params_.maxX_ = this->declare_parameter<double>("bbx.maxX", nan);
  if (std::isnan(params_.maxX_)) {
    RCLCPP_WARN(this->get_logger(), "No x-max value specified. Looking for bbx.maxX");
    ret = false;
  }
  params_.maxY_ = this->declare_parameter<double>("bbx.maxY", nan);
  if (std::isnan(params_.maxY_)) {
    RCLCPP_WARN(this->get_logger(), "No y-max value specified. Looking for bbx.maxY");
    ret = false;
  }
  params_.maxZ_ = this->declare_parameter<double>("bbx.maxZ", nan);
  if (std::isnan(params_.maxZ_)) {
    RCLCPP_WARN(this->get_logger(), "No z-max value specified. Looking for bbx.maxZ");
    ret = false;
  }

  params_.softBounds_ = this->declare_parameter<bool>("bbx.softBounds", false);
  params_.boundingBox_[0] = this->declare_parameter<double>("system.bbx.x", 0.5);
  params_.boundingBox_[1] = this->declare_parameter<double>("system.bbx.y", 0.5);
  params_.boundingBox_[2] = this->declare_parameter<double>("system.bbx.z", 0.3);
  params_.cuttoffIterations_ = this->declare_parameter<int>("nbvp.tree.cuttoff_iterations", 200);
  // zero_gain_: minimum bestGain_ to consider the tree "good enough" to stop
  // iterating. 0.0 means any single unmapped voxel satisfies gainFound() and
  // the while loop exits after exactly initIterations_ — the tree never grows
  // large. 0.5 is a meaningful threshold that requires multiple unmapped cells
  // to be visible before stopping, giving the planner time to build a richer
  // tree and pick a genuinely better frontier direction.
  params_.zero_gain_ = this->declare_parameter<double>("nbvp.gain.zero", 0.5);
  params_.dOvershoot_ = this->declare_parameter<double>("system.bbx.overshoot", 0.5);
  params_.log_ = this->declare_parameter<bool>("nbvp.log.on", false);
  params_.log_throttle_ = this->declare_parameter<double>("nbvp.log.throttle", 0.5);
  params_.navigationFrame_ = this->declare_parameter<std::string>("tf_frame", "world");
  params_.pcl_throttle_ = this->declare_parameter<double>("pcl_throttle", 0.333);
  params_.inspection_throttle_ = this->declare_parameter<double>("inspection_throttle", 0.25);
  params_.exact_root_ = this->declare_parameter<bool>("nbvp.tree.exact_root", true);

  return ret;
}

// insertPointcloudWithTf / insertPointcloudWithTfCamUp / insertPointcloudWithTfCamDown:
// DELETED ENTIRELY (Section 13.3, Open Issue #11). occupancy_map_cpp
// already owns all point-cloud insertion; nothing in exploration_cpp
// replaces these 1:1 - occupancyMapCallback() above does something
// different (read the finished map, not insert into it).

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::evasionCallback(
    const tello_autonomy_msgs::msg::Segment::SharedPtr segmentMsg)
{
  tree_->evade(*segmentMsg);
}

#endif  // NBVP_HPP_
