#ifndef _MESH_STRUCTURE_H_
#define _MESH_STRUCTURE_H_

#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/msg/marker.hpp>
#include <octomap_manager_shim/octomap_manager_shim.hpp>

namespace mesh {

class StlMesh
{
 public:
  StlMesh();
  StlMesh(std::fstream& file);
  StlMesh(const Eigen::Vector3d x1, const Eigen::Vector3d x2, const Eigen::Vector3d x3);
  ~StlMesh();
  static void setCameraParams(std::vector<double> cameraPitch,
                              std::vector<double> cameraHorizontalFoV,
                              std::vector<double> cameraVerticalFoV, double maxDist);
  static void setPeerPose(const geometry_msgs::msg::Pose& pose, int n_peer);
  static void setResolution(double resolution)
  {
    resolution_ = resolution;
  }
  static void setOctomapManager(octomap_manager_shim::OctomapManagerShim * manager)
  {
    manager_ = manager;
  }
  void incorporateViewFromPoseMsg(const geometry_msgs::msg::Pose& pose, int n_peer);
  double computeInspectableArea(const tf2::Transform& transform);
  void assembleMarkerArray(visualization_msgs::msg::Marker& inspected,
                           visualization_msgs::msg::Marker& uninspected) const;

 private:
  void incorporateViewFromTf(const tf2::Transform& transform, const std::vector<bool>& unobstructed);
  void split();
  bool collapse();
  bool getVisibility(const tf2::Transform& transform, bool& partialVisibility,
                     bool stop_at_unknown_cell, const std::vector<bool>& unobstructed = { }) const;

  bool isLeaf_;
  bool isHead_;
  bool isInspected_;
  std::vector<StlMesh*> children_;
  Eigen::Vector3d x1_;
  Eigen::Vector3d x2_;
  Eigen::Vector3d x3_;
  Eigen::Vector3d normal_;

  static double resolution_;
  static std::vector<double> cameraPitch_;
  static std::vector<double> cameraHorizontalFoV_;
  static std::vector<double> cameraVerticalFoV_;
  static double maxDist_;
  static std::vector<std::vector<tf2::Vector3> > camBoundNormals_;
  static octomap_manager_shim::OctomapManagerShim * manager_;
  static std::vector<tf2::Vector3> peer_vehicles_;
};
}

#endif // _MESH_STRUCTURE_H_
