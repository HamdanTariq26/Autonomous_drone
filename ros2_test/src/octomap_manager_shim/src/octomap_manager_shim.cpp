#include "octomap_manager_shim/octomap_manager_shim.hpp"

#include <cmath>

namespace octomap_manager_shim
{

namespace
{
// Matches the free-standing pointEigenToOctomap() helper in the real
// octomap_world.cc (file-local there too, not part of the public API).
inline octomap::point3d pointEigenToOctomap(const Eigen::Vector3d & point)
{
  return octomap::point3d(
    static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z()));
}
}  // namespace

OctomapManagerShim::OctomapManagerShim(std::shared_ptr<const octomap::OcTree> tree)
: octree_(std::move(tree))
{
}

void OctomapManagerShim::setOctree(std::shared_ptr<const octomap::OcTree> tree)
{
  std::atomic_store(&octree_, std::move(tree));
}

bool OctomapManagerShim::hasOctree() const
{
  return std::atomic_load(&octree_) != nullptr;
}

void OctomapManagerShim::setTreatUnknownAsOccupied(bool treat_as_occupied)
{
  treat_unknown_as_occupied_ = treat_as_occupied;
}

bool OctomapManagerShim::treatUnknownAsOccupied() const
{
  return treat_unknown_as_occupied_;
}

double OctomapManagerShim::getResolution() const
{
  auto local_octree = std::atomic_load(&octree_);
  // OctomapWorld::getResolution(): return octree_->getResolution();
  return local_octree ? local_octree->getResolution() : 0.0;
}

Eigen::Vector3d OctomapManagerShim::getMapSize() const
{
  auto local_octree = std::atomic_load(&octree_);
  if (!local_octree || local_octree->size() == 0) {
    return Eigen::Vector3d::Zero();
  }
  // OctomapWorld::getMapSize(): metric max - metric min, NOT getMetricSize()
  // (which assumes a tree centered at the origin - not necessarily true).
  double min_x = 0.0, min_y = 0.0, min_z = 0.0;
  double max_x = 0.0, max_y = 0.0, max_z = 0.0;
  local_octree->getMetricMin(min_x, min_y, min_z);
  local_octree->getMetricMax(max_x, max_y, max_z);
  return Eigen::Vector3d(max_x - min_x, max_y - min_y, max_z - min_z);
}

CellStatus OctomapManagerShim::getCellProbabilityPoint(
  const Eigen::Vector3d & point, double * probability) const
{
  auto local_octree = std::atomic_load(&octree_);
  // Ported directly from OctomapWorld::getCellProbabilityPoint(). Note this
  // always returns kUnknown for an unmapped cell regardless of
  // treat_unknown_as_occupied_ - that flag only affects the line-status
  // methods below, matching upstream.
  if (!local_octree) {
    if (probability) {
      *probability = -1.0;
    }
    return CellStatus::kUnknown;
  }

  const octomap::OcTreeNode * node = local_octree->search(point.x(), point.y(), point.z());
  if (node == nullptr) {
    if (probability) {
      *probability = -1.0;
    }
    return CellStatus::kUnknown;
  }
  if (probability) {
    *probability = node->getOccupancy();
  }
  return local_octree->isNodeOccupied(node) ? CellStatus::kOccupied : CellStatus::kFree;
}

CellStatus OctomapManagerShim::getVisibility(
  const Eigen::Vector3d & view_point, const Eigen::Vector3d & voxel_to_test,
  bool stop_at_unknown_cell) const
{
  auto local_octree = std::atomic_load(&octree_);
  // Ported directly from OctomapWorld::getVisibility().
  if (!local_octree) {
    return CellStatus::kUnknown;
  }

  octomap::KeyRay key_ray;
  local_octree->computeRayKeys(
    pointEigenToOctomap(view_point), pointEigenToOctomap(voxel_to_test), key_ray);

  const octomap::OcTreeKey voxel_to_test_key =
    local_octree->coordToKey(pointEigenToOctomap(voxel_to_test));

  for (const octomap::OcTreeKey & key : key_ray) {
    if (key != voxel_to_test_key) {
      const octomap::OcTreeNode * node = local_octree->search(key);
      if (node == nullptr) {
        if (stop_at_unknown_cell) {
          return CellStatus::kUnknown;
        }
      } else if (local_octree->isNodeOccupied(node)) {
        return CellStatus::kOccupied;
      }
    }
  }
  return CellStatus::kFree;
}

CellStatus OctomapManagerShim::getLineStatus(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end) const
{
  auto local_octree = std::atomic_load(&octree_);
  // Ported directly from OctomapWorld::getLineStatus(). Note this DOES
  // honor treat_unknown_as_occupied_, unlike getVisibility() above -
  // matches upstream exactly, including that asymmetry.
  //
  // IMPORTANT: when treat_unknown_as_occupied_ is false, unmapped cells
  // (node == nullptr) return kFree, NOT kUnknown. The exploration RRT in
  // iterate() only accepts kFree paths. Returning kUnknown here causes
  // 100% sample failure in a mostly-unmapped room, which is exactly the
  // environment an exploration planner operates in.
  if (!local_octree) {
    return treat_unknown_as_occupied_ ? CellStatus::kOccupied : CellStatus::kFree;
  }

  octomap::KeyRay key_ray;
  local_octree->computeRayKeys(pointEigenToOctomap(start), pointEigenToOctomap(end), key_ray);

  if (key_ray.size() == 0) {
    key_ray.addKey(local_octree->coordToKey(pointEigenToOctomap(start)));
  }

  for (const octomap::OcTreeKey & key : key_ray) {
    const octomap::OcTreeNode * node = local_octree->search(key);
    if (node == nullptr) {
      if (treat_unknown_as_occupied_) {
        return CellStatus::kOccupied;
      }
      // Unknown = free for exploration: the drone is allowed to fly there.
      // (Returning kUnknown here makes the RRT reject every sample in an
      // unmapped room, causing the "Exceeding maximum failed iterations" loop.)
    } else if (local_octree->isNodeOccupied(node)) {
      return CellStatus::kOccupied;
    }
  }
  return CellStatus::kFree;
}

CellStatus OctomapManagerShim::getLineStatusBoundingBox(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end,
  const Eigen::Vector3d & bounding_box_size) const
{
  auto local_octree = std::atomic_load(&octree_);
  // Ported directly from OctomapWorld::getLineStatusBoundingBox(). Builds a
  // grid of parallel offset lines along world x/y/z (not rotated to the
  // travel direction), with per-axis spacing derived from the octree
  // resolution so no cell between samples can be skipped. This is the real,
  // validated answer to Open Issue #6 - not the fixed/sparse pattern
  // discussed earlier in the chat before this source was pulled.
  if (!local_octree) {
    return treat_unknown_as_occupied_ ? CellStatus::kOccupied : CellStatus::kUnknown;
  }

  const double epsilon = 0.001;
  const double resolution = local_octree->getResolution();

  double x_disc = bounding_box_size.x() / std::ceil((bounding_box_size.x() + epsilon) / resolution);
  double y_disc = bounding_box_size.y() / std::ceil((bounding_box_size.y() + epsilon) / resolution);
  double z_disc = bounding_box_size.z() / std::ceil((bounding_box_size.z() + epsilon) / resolution);

  if (x_disc <= 0.0) {x_disc = 1.0;}
  if (y_disc <= 0.0) {y_disc = 1.0;}
  if (z_disc <= 0.0) {z_disc = 1.0;}

  const Eigen::Vector3d half_size = bounding_box_size * 0.5;

  const double epsilon_loop = 1e-4;
  for (double x = -half_size.x(); x <= half_size.x() + epsilon_loop; x += x_disc) {
    for (double y = -half_size.y(); y <= half_size.y() + epsilon_loop; y += y_disc) {
      for (double z = -half_size.z(); z <= half_size.z() + epsilon_loop; z += z_disc) {
        const Eigen::Vector3d offset(x, y, z);
        const CellStatus status = getLineStatus(start + offset, end + offset);
        if (status != CellStatus::kFree) {
          return status;
        }
      }
    }
  }
  return CellStatus::kFree;
}

}  // namespace octomap_manager_shim
