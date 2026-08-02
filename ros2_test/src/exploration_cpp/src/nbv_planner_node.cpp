#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <exploration_cpp/nbvp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto planner = std::make_shared<nbvInspection::nbvPlanner<Eigen::Matrix<double, 4, 1>>>(options);
  rclcpp::spin(planner);
  rclcpp::shutdown();
  return 0;
}
