#ifndef TREE_H_
#define TREE_H_

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tello_autonomy_msgs/msg/segment.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <octomap_manager_shim/octomap_manager_shim.hpp>
#include <exploration_cpp/mesh_structure.h>

namespace nbvInspection {

struct Params
{
  std::vector<double> camPitch_;
  std::vector<double> camHorizontal_;
  std::vector<double> camVertical_;
  std::vector<std::vector<Eigen::Vector3d> > camBoundNormals_;

  double igFree_;
  double igOccupied_;
  double igUnmapped_;
  double igArea_;
  double gainRange_;
  double degressiveCoeff_;
  double zero_gain_;

  double v_max_;
  double dyaw_max_;
  double dOvershoot_;
  double extensionRange_;
  bool exact_root_;
  int initIterations_;
  int cuttoffIterations_;
  double dt_;

  double minX_;
  double minY_;
  double minZ_;
  double maxX_;
  double maxY_;
  double maxZ_;
  bool softBounds_;
  Eigen::Vector3d boundingBox_;

  double meshResolution_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr inspectionPath_;
  std::string navigationFrame_;

  bool log_;
  double log_throttle_;
  double pcl_throttle_;
  double inspection_throttle_;
};

template<typename stateVec>
class Node
{
 public:
  Node();
  ~Node();
  stateVec state_;
  Node * parent_;
  std::vector<Node*> children_;
  double gain_;
  double distance_;
};

template<typename stateVec>
 class TreeBase
{
 protected:
  Params params_{};
  int counter_;
  double bestGain_;
  Node<stateVec> * bestNode_;
  Node<stateVec> * rootNode_;
  mesh::StlMesh * mesh_;
  octomap_manager_shim::OctomapManagerShim * manager_;
  stateVec root_;
  stateVec exact_root_;
  std::vector<std::vector<Eigen::Vector3d>*> segments_;
  std::vector<std::string> agentNames_;
 public:
  TreeBase();
  TreeBase(mesh::StlMesh * mesh, octomap_manager_shim::OctomapManagerShim * manager);
  ~TreeBase();
  virtual void setStateFromPoseMsg(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) = 0;
  virtual void setStateFromOdometryMsg(const nav_msgs::msg::Odometry& pose) = 0;
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::msg::PoseWithCovarianceStamped& pose, int n_peer) = 0;
  void setPeerStateFromPoseMsg1(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
  void setPeerStateFromPoseMsg2(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
  void setPeerStateFromPoseMsg3(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
  void evade(const tello_autonomy_msgs::msg::Segment& segmentMsg);
  virtual void iterate(int iterations) = 0;
  virtual void initialize() = 0;
  virtual std::vector<geometry_msgs::msg::Pose> getBestEdge(std::string targetFrame) = 0;
  virtual void clear() = 0;
  virtual std::vector<geometry_msgs::msg::Pose> getPathBackToPrevious(std::string targetFrame) = 0;
  virtual void memorizeBestBranch() = 0;
  void setParams(Params params);
  int getCounter();
  bool gainFound();
};
}

#endif
