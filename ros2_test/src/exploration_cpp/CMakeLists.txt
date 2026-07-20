cmake_minimum_required(VERSION 3.8)
project(exploration_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tello_autonomy_msgs REQUIRED)
find_package(octomap_manager_shim REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(nbv_planner_node
  src/nbv_planner_node.cpp
  src/mesh_structure.cpp
  src/rrt.cpp
  src/kdtree.c
  src/multiagent_collision_checker.cpp
)

target_include_directories(nbv_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nbv_planner_node
  rclcpp
  tello_autonomy_msgs
  octomap_manager_shim
  visualization_msgs
  octomap_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  ament_index_cpp
  nav_msgs
)

install(TARGETS nbv_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
