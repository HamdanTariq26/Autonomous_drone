#ifndef OCTOMAP_MANAGER_SHIM__OCTOMAP_MANAGER_SHIM_HPP_
#define OCTOMAP_MANAGER_SHIM__OCTOMAP_MANAGER_SHIM_HPP_

#include <memory>

#include <Eigen/Dense>
#include <octomap/octomap.h>

namespace octomap_manager_shim
{

// Matches volumetric_mapping::WorldBase::CellStatus exactly (see
// volumetric_map_base/include/volumetric_map_base/world_base.h in the real
// ethz-asl/volumetric_mapping repo - verified directly, not inferred).
enum class CellStatus
{
  kFree = 0,
  kOccupied = 1,
  kUnknown = 2
};

// Direct replacement for volumetric_mapping::OctomapManager (ARCHITECTURE.md
// Section 12.3), re-implemented against a plain octomap::OcTree so
// exploration_cpp/search_cpp don't need to port volumetric_mapping itself.
//
// Method signatures and algorithms here are ported directly from the real
// ethz-asl/volumetric_mapping OctomapWorld::getResolution/getMapSize/
// getCellProbabilityPoint/getVisibility/getLineStatus/getLineStatusBoundingBox
// (octomap_world/src/octomap_world.cc), not reinvented - verified against
// that source directly. Eigen::Vector3d is used throughout (rather than
// octomap::point3d) specifically so nbvplanner's Eigen-based call sites in
// rrt.cpp/mesh_structure.cpp port with minimal changes.
//
// Ownership: does NOT own the tree. Callers decode their own
// octomap::OcTree from TOPIC_OCCUPANCY_GRID (via octomap_msgs::msgToMap)
// and call setOctree() each time a fresh map arrives; this class only ever
// reads it.
class OctomapManagerShim
{
public:
  explicit OctomapManagerShim(std::shared_ptr<const octomap::OcTree> tree = nullptr);

  void setOctree(std::shared_ptr<const octomap::OcTree> tree);
  bool hasOctree() const;

  // Matches OctomapWorld::enableTreatUnknownAsOccupied()/
  // disableTreatUnknownAsOccupied(). Default true, matching upstream's
  // OctomapParameters default - i.e. collision-checking (getLineStatus /
  // getLineStatusBoundingBox) treats unmapped space as blocked unless you
  // explicitly turn this off. This is deliberately NOT the same thing as
  // "explore unknown space" - that incentive lives entirely in nbvplanner's
  // gain() function via igUnmapped_, not here. See the .cpp for more.
  void setTreatUnknownAsOccupied(bool treat_as_occupied);
  bool treatUnknownAsOccupied() const;

  // --- direct passthroughs ---
  double getResolution() const;

  // Empty-map guard: returns Vector3d::Zero() if the tree is null/empty,
  // matching the pattern nbvp.hpp uses (`manager_->getMapSize().norm() <=
  // 0.0`) rather than a separate bool return.
  Eigen::Vector3d getMapSize() const;

  CellStatus getCellProbabilityPoint(
    const Eigen::Vector3d & point, double * probability) const;

  // Is `voxel_to_test` visible from `view_point` - does a ray between them
  // pass through any occupied cell first? The voxel_to_test cell itself is
  // excluded from the check (matches upstream: you're asking "can I see
  // this cell", not "is this cell itself free").
  CellStatus getVisibility(
    const Eigen::Vector3d & view_point, const Eigen::Vector3d & voxel_to_test,
    bool stop_at_unknown_cell) const;

  // Single-line collision check (no bounding box). Public mainly because
  // getLineStatusBoundingBox is built from repeated calls to this; exposed
  // in case a caller wants a bare centerline check.
  CellStatus getLineStatus(const Eigen::Vector3d & start, const Eigen::Vector3d & end) const;

  // Is the straight edge from `start` to `end`, swept by a box of the given
  // size, collision-free? Ported directly from OctomapWorld::
  // getLineStatusBoundingBox: builds a grid of parallel offset lines along
  // world x/y/z, with per-axis spacing derived from the octree resolution
  // (ceil(box_size / resolution)) so no cell can be skipped between
  // samples - not a fixed/sparse sample count.
  CellStatus getLineStatusBoundingBox(
    const Eigen::Vector3d & start, const Eigen::Vector3d & end,
    const Eigen::Vector3d & bounding_box_size) const;

private:
  std::shared_ptr<const octomap::OcTree> octree_;
  bool treat_unknown_as_occupied_ = true;
};

}  // namespace octomap_manager_shim

#endif  // OCTOMAP_MANAGER_SHIM__OCTOMAP_MANAGER_SHIM_HPP_
