cmake_minimum_required(VERSION 3.8)
project(occupancy_map_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(tello_autonomy_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(occupancy_map_node
  src/occupancy_map_node.cpp
  src/main.cpp
)

ament_target_dependencies(occupancy_map_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  octomap_msgs
  octomap_ros
  message_filters
  pcl_conversions
  tello_autonomy_msgs
)

target_link_libraries(occupancy_map_node
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
  occupancy_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
