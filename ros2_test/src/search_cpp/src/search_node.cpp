// search_node.cpp  –  OMPL BIT* goal-directed planner for tello_autonomy
//
// Architecture note (project(v2).md Section 12.5):
//   This node is the "Go-To-Goal" layer. It is NOT a frontier explorer –
//   exploration_cpp (ported nbvplanner) handles that. This node is called
//   whenever the system needs to reach a specific, known destination:
//     - Return-to-Home
//     - User waypoint ("click-and-go")
//     - Pre-built map traversal (sequence of goal poses)
//
// Map input:  /tello_autonomy/occupancy_grid  (octomap_msgs/Octomap)
//   Produced by occupancy_map_cpp (Section 12.2). We subscribe here and
//   decode into a local octomap::OcTree, handed to octomap_manager_shim
//   (Section 12.3/13) for collision checking – same pattern as exploration_cpp.
//
// Service:    /tello_autonomy/search_plan  (tello_autonomy_msgs/srv/SearchPlan)
//   Client sends a start + goal PoseStamped; we return success + path[].
//
// Planner: BIT* (Batch Informed Trees) – user-selected. BIT* is an asymptotically
//   optimal informed sampling planner that converges faster than RRT* by
//   sampling from an informed ellipsoidal subset once an initial solution
//   is found. Appropriate for room-scale, low-obstacle-density environments.
//
// Bounding box (Section 12.5 open question, now answered):
//   Tello body 98 × 92.5 × 41 mm + 10 mm safety margin on each axis.
//   Every OMPL state is checked by sweeping the drone bbox through
//   getLineStatusBoundingBox() on a zero-length segment (point check).
//
// Threading: the map callback and the service handler can run on different
//   ROS2 executor threads. map_mutex_ guards the octree_/shim_ pair.

#include "search_cpp/search_node.hpp"

#include <Eigen/Dense>
#include <octomap_msgs/conversions.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>

namespace search_cpp
{

SearchNode::SearchNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("search_node", options)
{
  // --- parameters ---
  plan_timeout_sec_ = this->declare_parameter<double>("plan_timeout_sec", PLAN_TIMEOUT_SEC);

  // Drone bounding box: use drone body + margin (see header for rationale)
  const double bb_x = this->declare_parameter<double>("drone_bbox_x", DRONE_BB_X);
  const double bb_y = this->declare_parameter<double>("drone_bbox_y", DRONE_BB_Y);
  const double bb_z = this->declare_parameter<double>("drone_bbox_z", DRONE_BB_Z);
  drone_bbox_ = Eigen::Vector3d(bb_x, bb_y, bb_z);

  const std::string map_topic = this->declare_parameter<std::string>(
    "map_topic", "/tello_autonomy/occupancy_grid");

  // --- shim (no octree yet; will be set on first map message) ---
  shim_ = std::make_shared<octomap_manager_shim::OctomapManagerShim>();
  // Keep conservative default: unknown space = blocked for path interiors.
  // Start/goal endpoints are handled separately via isEndpointAcceptable().
  shim_->setTreatUnknownAsOccupied(true);

  // --- map subscriber ---
  map_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    map_topic, rclcpp::QoS(10),
    std::bind(&SearchNode::mapCallback, this, std::placeholders::_1));

  // --- service server ---
  plan_srv_ = this->create_service<tello_autonomy_msgs::srv::SearchPlan>(
    "/tello_autonomy/search_plan",
    std::bind(
      &SearchNode::planCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    this->get_logger(),
    "search_node started: planner=BIT* timeout=%.2fs "
    "drone_bbox=[%.3f, %.3f, %.3f]m",
    plan_timeout_sec_, bb_x, bb_y, bb_z);
}

// ---------------------------------------------------------------------------
// Map callback: decode octomap_msgs → octomap::OcTree → shim
// ---------------------------------------------------------------------------
void SearchNode::mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  // msgToMap returns a raw AbstractOcTree* - we own it
  auto * raw = octomap_msgs::msgToMap(*msg);
  if (!raw) {
    RCLCPP_WARN(this->get_logger(), "mapCallback: msgToMap returned null, skipping");
    return;
  }

  auto * raw_octree = dynamic_cast<octomap::OcTree *>(raw);
  if (!raw_octree) {
    RCLCPP_WARN(
      this->get_logger(),
      "mapCallback: received map is not an OcTree (type=%s), skipping",
      msg->id.c_str());
    delete raw;
    return;
  }

  auto new_tree = std::shared_ptr<const octomap::OcTree>(raw_octree);
  std::lock_guard<std::mutex> lock(map_mutex_);
  octree_ = new_tree;
  shim_->setOctree(octree_);
}

// ---------------------------------------------------------------------------
// State validity checker (called by OMPL's SpaceInformation)
// ---------------------------------------------------------------------------
bool SearchNode::isStateValid(
  const ob::State * state,
  const std::shared_ptr<const octomap_manager_shim::OctomapManagerShim> & shim_snap,
  bool allow_unknown) const
{
  if (!shim_snap->hasOctree()) {
    return false;  // no map yet – treat everything as invalid (conservative)
  }

  const auto * se3 = state->as<ob::SE3StateSpace::StateType>();
  const Eigen::Vector3d pos(se3->getX(), se3->getY(), se3->getZ());

  const double resolution = shim_snap->getResolution();
  if (resolution <= 0.0) return false;

  // Build the same grid that getLineStatusBoundingBox would, but call
  // getCellProbabilityPoint directly — no zero-length ray involved.
  const double eps = 0.001;
  const Eigen::Vector3d half = drone_bbox_ * 0.5;

  double dx = drone_bbox_.x() / std::ceil((drone_bbox_.x() + eps) / resolution);
  double dy = drone_bbox_.y() / std::ceil((drone_bbox_.y() + eps) / resolution);
  double dz = drone_bbox_.z() / std::ceil((drone_bbox_.z() + eps) / resolution);
  if (dx <= 0.0) dx = resolution;
  if (dy <= 0.0) dy = resolution;
  if (dz <= 0.0) dz = resolution;

  for (double x = -half.x(); x <= half.x() + eps; x += dx) {
    for (double y = -half.y(); y <= half.y() + eps; y += dy) {
      for (double z = -half.z(); z <= half.z() + eps; z += dz) {
        const auto status = shim_snap->getCellProbabilityPoint(
          pos + Eigen::Vector3d(x, y, z), nullptr);
        if (status == octomap_manager_shim::CellStatus::kOccupied) {
          return false;
        }
        // kUnknown with treat_unknown_as_occupied=true: getCellProbabilityPoint
        // does NOT honor that flag (by design — see shim.hpp comment). We
        // enforce it here explicitly unless allow_unknown is true (used near endpoints).
        if (!allow_unknown && status == octomap_manager_shim::CellStatus::kUnknown) {
          return false;  // conservative: unknown = blocked for interior states
        }
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Plan service handler
// ---------------------------------------------------------------------------
void SearchNode::planCallback(
  const std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan::Request> req,
  std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan::Response> res)
{
  // Take a shared snapshot of the shim so the map can be updated concurrently
  // while we are planning without holding map_mutex_ for the full duration.
  std::shared_ptr<const octomap_manager_shim::OctomapManagerShim> shim_snap;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!shim_ || !shim_->hasOctree()) {
      RCLCPP_WARN(this->get_logger(), "SearchPlan called before any map received");
      res->success = false;
      return;
    }
    // Snapshot: create a new shim sharing the same (immutable) octree ptr
    auto snap = std::make_shared<octomap_manager_shim::OctomapManagerShim>();
    snap->setOctree(octree_);
    snap->setTreatUnknownAsOccupied(true);  // conservative: interior path checks block on unknown
    shim_snap = snap;
  }

  // --- 1. Determine search bounds from the live map ---
  const Eigen::Vector3d map_size = shim_snap->getMapSize();
  if (map_size.norm() <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Map is empty, cannot plan");
    res->success = false;
    return;
  }

  // --- 2. Set up OMPL SE3 state space ---
  auto space = std::make_shared<ob::SE3StateSpace>();

  // Bounds: use the actual OcTree metric bounds so OMPL doesn't sample outside
  // the mapped volume. shim_ exposes getMapSize() but not min/max separately,
  // so we read those directly from the octree once (with the lock).
  double min_x, min_y, min_z, max_x, max_y, max_z;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
  }

  // Ensure bounds encompass both the start and goal positions with a safe margin,
  // otherwise OMPL will immediately reject them if they lie slightly outside the 
  // current mapped volume.
  const double margin = 1.0;
  min_x = std::min({min_x, req->start.pose.position.x, req->goal.pose.position.x}) - margin;
  min_y = std::min({min_y, req->start.pose.position.y, req->goal.pose.position.y}) - margin;
  min_z = std::min({min_z, req->start.pose.position.z, req->goal.pose.position.z}) - margin;
  
  max_x = std::max({max_x, req->start.pose.position.x, req->goal.pose.position.x}) + margin;
  max_y = std::max({max_y, req->start.pose.position.y, req->goal.pose.position.y}) + margin;
  max_z = std::max({max_z, req->start.pose.position.z, req->goal.pose.position.z}) + margin;

  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, min_x);  bounds.setHigh(0, max_x);
  bounds.setLow(1, min_y);  bounds.setHigh(1, max_y);
  bounds.setLow(2, min_z);  bounds.setHigh(2, max_z);
  space->setBounds(bounds);

  // Pre-calculate start and goal positions to use in the validity checker
  const Eigen::Vector3d start_pos(req->start.pose.position.x, req->start.pose.position.y, req->start.pose.position.z);
  const Eigen::Vector3d goal_pos(req->goal.pose.position.x, req->goal.pose.position.y, req->goal.pose.position.z);

  // --- 3. SpaceInformation + validity checker ---
  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker(
    [this, shim_snap, start_pos, goal_pos](const ob::State * s) {
      // For states very close to the start or goal (< 0.2m), we allow kUnknown space.
      // This is necessary because OMPL strictly evaluates the exact start and goal
      // states against this checker, and they are often unscanned (e.g. launch point).
      // If we don't allow unknown here, OMPL will instantly reject the start/goal 
      // and refuse to plan. The rest of the path is strictly constrained.
      const auto * se3 = s->as<ob::SE3StateSpace::StateType>();
      Eigen::Vector3d pos(se3->getX(), se3->getY(), se3->getZ());
      bool is_near_endpoint = (pos - start_pos).norm() < 0.2 || (pos - goal_pos).norm() < 0.2;
      return this->isStateValid(s, shim_snap, is_near_endpoint);
    });
  si->setup();

  // --- 4. Start and goal states ---
  ob::ScopedState<ob::SE3StateSpace> start(space);
  start->setX(req->start.pose.position.x);
  start->setY(req->start.pose.position.y);
  start->setZ(req->start.pose.position.z);
  start->rotation().x = req->start.pose.orientation.x;
  start->rotation().y = req->start.pose.orientation.y;
  start->rotation().z = req->start.pose.orientation.z;
  start->rotation().w = req->start.pose.orientation.w;

  ob::ScopedState<ob::SE3StateSpace> goal(space);
  goal->setX(req->goal.pose.position.x);
  goal->setY(req->goal.pose.position.y);
  goal->setZ(req->goal.pose.position.z);
  goal->rotation().x = req->goal.pose.orientation.x;
  goal->rotation().y = req->goal.pose.orientation.y;
  goal->rotation().z = req->goal.pose.orientation.z;
  goal->rotation().w = req->goal.pose.orientation.w;

  // Validate start/goal using a narrow check before handing to OMPL.
  // Unknown cells (never scanned) are accepted.
  // Only hard-reject if they are explicitly KNOWN to be occupied (inside a wall).
  if (!isEndpointAcceptable(start_pos, shim_snap)) {
    RCLCPP_WARN(this->get_logger(), "SearchPlan: start state is in a known-occupied cell");
    res->success = false;
    return;
  }
  if (!isEndpointAcceptable(goal_pos, shim_snap)) {
    RCLCPP_WARN(this->get_logger(), "SearchPlan: goal state is in a known-occupied cell");
    res->success = false;
    return;
  }

  // --- 5. Set up BIT* planner via SimpleSetup ---
  og::SimpleSetup ss(si);
  ss.setStartAndGoalStates(start, goal);

  // Optimization objective: minimize path length (Section 12.5: shortest safe path)
  ss.setOptimizationObjective(
    std::make_shared<ob::PathLengthOptimizationObjective>(si));

  // BIT* (Batch Informed Trees) – selected per user request
  ss.setPlanner(std::make_shared<og::BITstar>(si));

  // --- 6. Solve ---
  const ob::PlannerStatus status = ss.solve(plan_timeout_sec_);

  if (!status || !ss.haveExactSolutionPath()) {
    RCLCPP_WARN(
      this->get_logger(),
      "SearchPlan: BIT* found no solution within %.2fs", plan_timeout_sec_);
    res->success = false;
    return;
  }

  // --- 7. Simplify/smooth the raw path ---
  // OMPL's PathSimplifier removes unnecessary waypoints and smooths corners.
  // This is especially important for BIT* whose raw paths can be slightly
  // jagged even after optimization.
  ss.simplifySolution();
  og::PathGeometric & path = ss.getSolutionPath();
  path.interpolate();  // add intermediate states for smooth following

  // --- 8. Convert to ROS2 message ---
  const std::string frame_id =
    req->start.header.frame_id.empty() ? "map" : req->start.header.frame_id;
  res->path = omplPathToMsg(path, frame_id, this->now());
  res->success = true;

  RCLCPP_INFO(
    this->get_logger(),
    "SearchPlan: found path with %zu waypoints (%.2fs budget)",
    res->path.size(), plan_timeout_sec_);
}

// ---------------------------------------------------------------------------
// Convert OMPL path states → PoseStamped[]
// ---------------------------------------------------------------------------
std::vector<geometry_msgs::msg::PoseStamped> SearchNode::omplPathToMsg(
  const og::PathGeometric & path,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  std::vector<geometry_msgs::msg::PoseStamped> out;
  out.reserve(path.getStateCount());

  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    const auto * se3 = path.getState(i)->as<ob::SE3StateSpace::StateType>();
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = frame_id;
    ps.pose.position.x = se3->getX();
    ps.pose.position.y = se3->getY();
    ps.pose.position.z = se3->getZ();
    ps.pose.orientation.x = se3->rotation().x;
    ps.pose.orientation.y = se3->rotation().y;
    ps.pose.orientation.z = se3->rotation().z;
    ps.pose.orientation.w = se3->rotation().w;
    out.push_back(ps);
  }

  return out;
}

// ---------------------------------------------------------------------------
// Endpoint acceptability check
// ---------------------------------------------------------------------------
bool SearchNode::isEndpointAcceptable(
  const Eigen::Vector3d & pos,
  const std::shared_ptr<const octomap_manager_shim::OctomapManagerShim> & shim) const
{
  // Accept if the position is free or unknown (never scanned).
  // Only hard-reject positions that are KNOWN to be occupied — e.g. inside a wall.
  // This lets the drone plan to its launch point (which may be unscanned)
  // without opening interior path states to unknown space.
  const auto status = shim->getCellProbabilityPoint(pos, nullptr);
  return (status != octomap_manager_shim::CellStatus::kOccupied);
}

}  // namespace search_cpp

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<search_cpp::SearchNode>());
  rclcpp::shutdown();
  return 0;
}
