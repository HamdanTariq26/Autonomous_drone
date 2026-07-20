#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_

#include <cstdlib>
#include <exploration_cpp/multiagent_collision_checker.h>
#include <exploration_cpp/rrt.h>
#include <exploration_cpp/tree.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/utils.h>

nbvInspection::RrtTree::RrtTree(rclcpp::Node * node)
    : nbvInspection::TreeBase<StateVec>::TreeBase(), node_(node)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(node_->now().seconds());
  }

  bool ifLog = false;
  node_->get_parameter_or("nbvp.log.on", ifLog, false);
  if (ifLog) {
    time_t rawtime;
    struct tm * ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ = ament_index_cpp::get_package_share_directory("exploration_cpp") + "/data/"
        + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
        + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
        + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

nbvInspection::RrtTree::RrtTree(mesh::StlMesh * mesh, octomap_manager_shim::OctomapManagerShim * manager, rclcpp::Node * node)
    : nbvInspection::TreeBase<StateVec>::TreeBase(mesh, manager), node_(node)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(node_->now().seconds());
  }

  bool ifLog = false;
  node_->get_parameter_or("nbvp.log.on", ifLog, false);
  if (ifLog) {
    time_t rawtime;
    struct tm * ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ = ament_index_cpp::get_package_share_directory("exploration_cpp") + "/data/"
        + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
        + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
        + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

nbvInspection::RrtTree::~RrtTree()
{
  delete rootNode_;
  kd_free(kdTree_);
  if (fileResponse_.is_open()) {
    fileResponse_.close();
  }
  if (fileTree_.is_open()) {
    fileTree_.close();
  }
  if (filePath_.is_open()) {
    filePath_.close();
  }
}

void nbvInspection::RrtTree::setStateFromPoseMsg(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose)
{
  geometry_msgs::msg::TransformStamped transformMsg;
  try {
    transformMsg = tf_buffer_->lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return;
  }
  tf2::Transform transform;
  tf2::fromMsg(transformMsg.transform, transform);
  
  tf2::Transform poseTF;
  tf2::fromMsg(pose.pose.pose, poseTF);
  
  tf2::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf2::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  root_[3] = yaw;

  static double logThrottleTime = node_->now().seconds();
  if (node_->now().seconds() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
  if (node_->now().seconds() - inspectionThrottleTime_[0] > params_.inspection_throttle_) {
    inspectionThrottleTime_[0] += params_.inspection_throttle_;
    if (mesh_) {
      geometry_msgs::msg::Pose poseTransformed;
      tf2::toMsg(transform * poseTF, poseTransformed);
      mesh_->setPeerPose(poseTransformed, 0);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
      visualization_msgs::msg::Marker inspected;
      inspected.ns = "meshInspected";
      inspected.id = 0;
      inspected.header.stamp = pose.header.stamp;
      inspected.header.frame_id = params_.navigationFrame_;
      inspected.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      inspected.action = visualization_msgs::msg::Marker::ADD;
      inspected.pose.position.x = 0.0;
      inspected.pose.position.y = 0.0;
      inspected.pose.position.z = 0.0;
      inspected.pose.orientation.x = 0.0;
      inspected.pose.orientation.y = 0.0;
      inspected.pose.orientation.z = 0.0;
      inspected.pose.orientation.w = 1.0;
      inspected.scale.x = 1.0;
      inspected.scale.y = 1.0;
      inspected.scale.z = 1.0;
      visualization_msgs::msg::Marker uninspected = inspected;
      uninspected.id++;
      uninspected.ns = "meshUninspected";
      mesh_->assembleMarkerArray(inspected, uninspected);
      if (inspected.points.size() > 0) {
        params_.inspectionPath_->publish(inspected);
      }
      if (uninspected.points.size() > 0) {
        params_.inspectionPath_->publish(uninspected);
      }
    }
  }
}

void nbvInspection::RrtTree::setStateFromOdometryMsg(
    const nav_msgs::msg::Odometry& pose)
{
  geometry_msgs::msg::TransformStamped transformMsg;
  try {
    transformMsg = tf_buffer_->lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return;
  }
  tf2::Transform transform;
  tf2::fromMsg(transformMsg.transform, transform);
  
  tf2::Transform poseTF;
  tf2::fromMsg(pose.pose.pose, poseTF);
  
  tf2::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf2::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  root_[3] = yaw;

  static double logThrottleTime = node_->now().seconds();
  if (node_->now().seconds() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
  if (node_->now().seconds() - inspectionThrottleTime_[0] > params_.inspection_throttle_) {
    inspectionThrottleTime_[0] += params_.inspection_throttle_;
    if (mesh_) {
      geometry_msgs::msg::Pose poseTransformed;
      tf2::toMsg(transform * poseTF, poseTransformed);
      mesh_->setPeerPose(poseTransformed, 0);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
      visualization_msgs::msg::Marker inspected;
      inspected.ns = "meshInspected";
      inspected.id = 0;
      inspected.header.stamp = pose.header.stamp;
      inspected.header.frame_id = params_.navigationFrame_;
      inspected.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      inspected.action = visualization_msgs::msg::Marker::ADD;
      inspected.pose.position.x = 0.0;
      inspected.pose.position.y = 0.0;
      inspected.pose.position.z = 0.0;
      inspected.pose.orientation.x = 0.0;
      inspected.pose.orientation.y = 0.0;
      inspected.pose.orientation.z = 0.0;
      inspected.pose.orientation.w = 1.0;
      inspected.scale.x = 1.0;
      inspected.scale.y = 1.0;
      inspected.scale.z = 1.0;
      visualization_msgs::msg::Marker uninspected = inspected;
      uninspected.id++;
      uninspected.ns = "meshUninspected";
      mesh_->assembleMarkerArray(inspected, uninspected);
      if (inspected.points.size() > 0) {
        params_.inspectionPath_->publish(inspected);
      }
      if (uninspected.points.size() > 0) {
        params_.inspectionPath_->publish(uninspected);
      }
    }
  }
}

void nbvInspection::RrtTree::setPeerStateFromPoseMsg(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose, int n_peer)
{
  geometry_msgs::msg::TransformStamped transformMsg;
  try {
    transformMsg = tf_buffer_->lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return;
  }
  tf2::Transform transform;
  tf2::fromMsg(transformMsg.transform, transform);
  
  tf2::Transform poseTF;
  tf2::fromMsg(pose.pose.pose, poseTF);
  geometry_msgs::msg::Pose poseTransformed;
  tf2::toMsg(transform * poseTF, poseTransformed);
  if (node_->now().seconds() - inspectionThrottleTime_[n_peer] > params_.inspection_throttle_) {
    inspectionThrottleTime_[n_peer] += params_.inspection_throttle_;
    if (mesh_) {
      mesh_->setPeerPose(poseTransformed, n_peer);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, n_peer);
    }
  }
}

void nbvInspection::RrtTree::iterate(int iterations)
{
  StateVec newState;
  double radius = sqrt(
      SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_)
      + SQ(params_.minZ_ - params_.maxZ_));
  bool solutionFound = false;
  while (!solutionFound) {
    for (int i = 0; i < 3; i++) {
      newState[i] = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
      continue;
    // Use root_ (live drone position updated by every pose callback) rather
    // than rootNode_->state_ (set once at initialize() and never updated
    // mid-cycle). rootNode_->state_ may be the previous waypoint, not where
    // the drone actually is right now — especially important with exact_root_.
    newState += root_;
    if (!params_.softBounds_) {
      if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
        continue;
      } else if (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
        continue;
      }
    }
    solutionFound = true;
  }

  kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }
  nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(
      nearest);
  kd_res_free(nearest);

  Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                            newState[2] - origin[2]);
  if (direction.norm() > params_.extensionRange_) {
    direction = params_.extensionRange_ * direction.normalized();
  }
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];
  if (octomap_manager_shim::CellStatus::kFree
      == manager_->getLineStatusBoundingBox(
          origin, direction + origin + direction.normalized() * params_.dOvershoot_,
          params_.boundingBox_)
      && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_, segments_)) {
    newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);
    newNode->gain_ = newParent->gain_
        + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * newNode->distance_);

    kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

    publishNode(newNode);

    if (newNode->gain_ > bestGain_) {
      bestGain_ = newNode->gain_;
      bestNode_ = newNode;
    }
    counter_++;
  }
}

void nbvInspection::RrtTree::initialize()
{
  g_ID_ = 0;
  int i;
  for (i = 0; i < agentNames_.size(); i++) {
    if (agentNames_[i].compare(params_.navigationFrame_) == 0) {
      break;
    }
  }
  if (i < agentNames_.size()) {
    segments_[i]->clear();
  }
  kdTree_ = kd_create(3);

  if (params_.log_) {
    if (fileTree_.is_open()) {
      fileTree_.close();
    }
    fileTree_.open((logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt").c_str(),
                   std::ios::out);
  }

  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = NULL;

  if (params_.exact_root_) {
    if (iterationCount_ <= 1) {
      exact_root_ = root_;
    }
    rootNode_->state_ = exact_root_;
  } else {
    rootNode_->state_ = root_;
  }
  kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(),
             rootNode_);
  iterationCount_++;

  for (typename std::vector<StateVec>::reverse_iterator iter = bestBranchMemory_.rbegin();
      iter != bestBranchMemory_.rend(); ++iter) {
    StateVec newState = *iter;
    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      continue;
    }
    nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(
        nearest);
    kd_res_free(nearest);

    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                              newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_) {
      direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];
    if (octomap_manager_shim::CellStatus::kFree
        == manager_->getLineStatusBoundingBox(
            origin, direction + origin + direction.normalized() * params_.dOvershoot_,
            params_.boundingBox_)
        && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_,
                                      segments_)) {
      nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ = newParent->gain_
          + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * newNode->distance_);

      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      publishNode(newNode);

      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
  }

  visualization_msgs::msg::Marker p;
  p.header.stamp = node_->now();
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "workspace";
  p.type = visualization_msgs::msg::Marker::CUBE;
  p.action = visualization_msgs::msg::Marker::ADD;
  p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
  p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
  p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
  tf2::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = params_.maxX_ - params_.minX_;
  p.scale.y = params_.maxY_ - params_.minY_;
  p.scale.z = params_.maxZ_ - params_.minZ_;
  p.color.r = 200.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 0.1;
  p.frame_locked = false;
  params_.inspectionPath_->publish(p);
}

std::vector<geometry_msgs::msg::Pose> nbvInspection::RrtTree::getBestEdge(std::string targetFrame)
{
  std::vector<geometry_msgs::msg::Pose> ret;
  nbvInspection::Node<StateVec> * current = bestNode_;
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      current = current->parent_;
    }
    ret = samplePath(current->parent_->state_, current->state_, targetFrame);
    history_.push(current->parent_->state_);
    exact_root_ = current->state_;
  }
  return ret;
}

double nbvInspection::RrtTree::gain(StateVec state)
{
  double gain = 0.0;
  const double disc = manager_->getResolution();
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  double rangeSq = pow(params_.gainRange_, 2.0);
  for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
      vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc) {
    for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
        vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc) {
      for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
          vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc) {
        Eigen::Vector3d dir = vec - origin;
        if (dir.transpose().dot(dir) > rangeSq) {
          continue;
        }
        
        // Convert dir from SLAM frame (Z-fwd, X-right, Y-down) 
        // to ROS frame (X-fwd, Y-left, Z-up) which the frustum expects
        Eigen::Vector3d dir_ros(dir.z(), -dir.x(), -dir.y());

        bool insideAFieldOfView = false;
        for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = params_
            .camBoundNormals_.begin(); itCBN != params_.camBoundNormals_.end(); itCBN++) {
          bool inThisFieldOfView = true;
          for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin();
              itSingleCBN != itCBN->end(); itSingleCBN++) {
            
            // state[3] is yaw in SLAM frame (positive = turn right = -Z in ROS)
            // So we rotate the frustum around UnitZ by -state[3]
            Eigen::Vector3d normal = Eigen::AngleAxisd(-state[3], Eigen::Vector3d::UnitZ())
                * (*itSingleCBN);
            
            // Compare the ROS direction with the ROS normal
            double val = dir_ros.dot(normal.normalized());
            if (val < SQRT2 * disc) {
              inThisFieldOfView = false;
              break;
            }
          }
          if (inThisFieldOfView) {
            insideAFieldOfView = true;
            break;
          }
        }
        if (!insideAFieldOfView) {
          continue;
        }
        double probability;
        octomap_manager_shim::CellStatus node = manager_->getCellProbabilityPoint(
            vec, &probability);
        if (node == octomap_manager_shim::CellStatus::kUnknown) {
          if (octomap_manager_shim::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igUnmapped_;
          }
        } else if (node == octomap_manager_shim::CellStatus::kOccupied) {
          if (octomap_manager_shim::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igOccupied_;
          }
        } else {
          if (octomap_manager_shim::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igFree_;
          }
        }
      }
    }
  }
  gain *= pow(disc, 3.0);
  if (mesh_) {
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(state.x(), state.y(), state.z()));
    tf2::Quaternion quaternion;
    quaternion.setEuler(0.0, 0.0, state[3]);
    transform.setRotation(quaternion);
    gain += params_.igArea_ * mesh_->computeInspectableArea(transform);
  }
  return gain;
}

std::vector<geometry_msgs::msg::Pose> nbvInspection::RrtTree::getPathBackToPrevious(
    std::string targetFrame)
{
  std::vector<geometry_msgs::msg::Pose> ret;
  if (history_.empty()) {
    return ret;
  }
  ret = samplePath(root_, history_.top(), targetFrame);
  history_.pop();
  return ret;
}

void nbvInspection::RrtTree::memorizeBestBranch()
{
  bestBranchMemory_.clear();
  Node<StateVec> * current = bestNode_;
  while (current->parent_ && current->parent_->parent_) {
    bestBranchMemory_.push_back(current->state_);
    current = current->parent_;
  }
}

void nbvInspection::RrtTree::clear()
{
  delete rootNode_;
  rootNode_ = NULL;

  counter_ = 0;
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;

  kd_free(kdTree_);
}

void nbvInspection::RrtTree::publishNode(Node<StateVec> * node)
{
  visualization_msgs::msg::Marker p;
  p.header.stamp = node_->now();
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::msg::Marker::ARROW;
  p.action = visualization_msgs::msg::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf2::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::max(node->gain_ / 20.0, 0.05);
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0 / 255.0;
  p.color.g = 167.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.frame_locked = false;
  params_.inspectionPath_->publish(p);

  if (!node->parent_)
    return;

  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_branches";
  p.type = visualization_msgs::msg::Marker::ARROW;
  p.action = visualization_msgs::msg::Marker::ADD;
  p.pose.position.x = node->parent_->state_[0];
  p.pose.position.y = node->parent_->state_[1];
  p.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 0.03;
  p.scale.z = 0.03;
  p.color.r = 100.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.7;
  p.color.a = 1.0;
  p.frame_locked = false;
  params_.inspectionPath_->publish(p);

  if (params_.log_) {
    for (int i = 0; i < node->state_.size(); i++) {
      fileTree_ << node->state_[i] << ",";
    }
    fileTree_ << node->gain_ << ",";
    for (int i = 0; i < node->parent_->state_.size(); i++) {
      fileTree_ << node->parent_->state_[i] << ",";
    }
    fileTree_ << node->parent_->gain_ << "\n";
  }
}

std::vector<geometry_msgs::msg::Pose> nbvInspection::RrtTree::samplePath(StateVec start, StateVec end,
                                                                    std::string targetFrame)
{
  std::vector<geometry_msgs::msg::Pose> ret;
  geometry_msgs::msg::TransformStamped transformMsg;
  try {
    transformMsg = tf_buffer_->lookupTransform(targetFrame, params_.navigationFrame_, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return ret;
  }
  tf2::Transform transform;
  tf2::fromMsg(transformMsg.transform, transform);
  
  Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1], end[2] - start[2]);
  double yaw_direction = end[3] - start[3];
  if (yaw_direction > M_PI) {
    yaw_direction -= 2.0 * M_PI;
  }
  if (yaw_direction < -M_PI) {
    yaw_direction += 2.0 * M_PI;
  }
  double d_norm = distance.norm();
  double y_abs = std::abs(yaw_direction);

  if (d_norm < 1e-6 && y_abs < 1e-6) {
    // Degenerate edge, skip interpolation and just return the start pose
    tf2::Quaternion quat;
    quat.setEuler(0.0, 0.0, start[3]);
    tf2::Vector3 origin(start[0], start[1], start[2]);
    origin = transform * origin;
    quat = transform * quat;
    tf2::Transform poseTF(quat, origin);
    geometry_msgs::msg::Pose pose;
    tf2::toMsg(poseTF, pose);
    ret.push_back(pose);
    return ret;
  }

  double t_dist = (d_norm > 1e-6) ? (params_.dt_ * params_.v_max_ / d_norm) : std::numeric_limits<double>::infinity();
  double t_yaw = (y_abs > 1e-6) ? (params_.dt_ * params_.dyaw_max_ / y_abs) : std::numeric_limits<double>::infinity();
  double disc = std::min(t_dist, t_yaw);
  
  // Guard against infinite loops or NaNs
  if (disc <= 0.0 || std::isinf(disc) || std::isnan(disc)) {
    disc = 1.0;
  }
  for (double it = 0.0; it <= 1.0; it += disc) {
    tf2::Vector3 origin((1.0 - it) * start[0] + it * end[0], (1.0 - it) * start[1] + it * end[1],
                       (1.0 - it) * start[2] + it * end[2]);
    double yaw = start[3] + yaw_direction * it;
    if (yaw > M_PI)
      yaw -= 2.0 * M_PI;
    if (yaw < -M_PI)
      yaw += 2.0 * M_PI;
    tf2::Quaternion quat;
    // Set quaternion in SLAM frame (rotation around Y-axis)
    quat.setX(0.0);
    quat.setY(sin(yaw / 2.0));
    quat.setZ(0.0);
    quat.setW(cos(yaw / 2.0));
    origin = transform * origin;
    quat = transform * quat;
    tf2::Transform poseTF(quat, origin);
    geometry_msgs::msg::Pose pose;
    tf2::toMsg(poseTF, pose);
    ret.push_back(pose);
    if (params_.log_) {
      filePath_ << poseTF.getOrigin().x() << ",";
      filePath_ << poseTF.getOrigin().y() << ",";
      filePath_ << poseTF.getOrigin().z() << ",";
      
      double rp, pp, yp;
      tf2::Matrix3x3(poseTF.getRotation()).getRPY(rp, pp, yp);
      filePath_ << yp << "\n";
    }
  }
  return ret;
}

#endif
