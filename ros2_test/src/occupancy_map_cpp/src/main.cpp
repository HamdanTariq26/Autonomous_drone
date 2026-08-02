#include <rclcpp/rclcpp.hpp>

#include "occupancy_map_cpp/occupancy_map_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<occupancy_map_cpp::OccupancyMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
