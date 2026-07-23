#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stack>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <kdtree/kdtree.h>
#include <exploration_cpp/tree.h>
#include <exploration_cpp/mesh_structure.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

class RrtTree : public TreeBase<Eigen::Vector4d>
{
 public:
  typedef Eigen::Vector4d StateVec;

  RrtTree(rclcpp::Node * node);
  RrtTree(mesh::StlMesh * mesh, octomap_manager_shim::OctomapManagerShim * manager, rclcpp::Node * node);
  ~RrtTree();
  virtual void setStateFromPoseMsg(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
  virtual void setStateFromOdometryMsg(const nav_msgs::msg::Odometry& pose);
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::msg::PoseWithCovarianceStamped& pose, int n_peer);
  virtual void initialize();
  virtual void iterate(int iterations);
  virtual std::vector<geometry_msgs::msg::Pose> getBestEdge(std::string targetFrame);
  virtual void clear();
  virtual std::vector<geometry_msgs::msg::Pose> getPathBackToPrevious(std::string targetFrame);
  virtual void memorizeBestBranch();
  void publishNode(Node<StateVec> * node);
  double gain(StateVec state);
  std::vector<geometry_msgs::msg::Pose> samplePath(StateVec start, StateVec end,
                                                   std::string targetFrame);
  void setLiveSlamPoints(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
 protected:
  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  kdtree * kdTree_;
  std::stack<StateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
  std::vector<double> inspectionThrottleTime_;
  std::vector<Eigen::Vector3d> liveSlamPoints_;
  // History of positions the drone has actually flown to, used to penalize
  // re-visiting already-explored areas in gain().
  std::vector<Eigen::Vector3d> visitedPositions_;
};
}

#endif
