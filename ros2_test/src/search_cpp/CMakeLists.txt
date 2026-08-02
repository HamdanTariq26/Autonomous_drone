cmake_minimum_required(VERSION 3.8)
project(search_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- find dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap_manager_shim REQUIRED)
find_package(tello_autonomy_msgs REQUIRED)
find_package(ompl REQUIRED)

# ---- executable ----
add_executable(search_node
  src/search_node.cpp
)

target_include_directories(search_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(search_node
  ${OCTOMAP_LIBRARIES}
  ${OMPL_LIBRARIES}
)

ament_target_dependencies(search_node
  rclcpp
  geometry_msgs
  nav_msgs
  octomap_msgs
  octomap_manager_shim
  tello_autonomy_msgs
)

# ---- install ----
install(TARGETS search_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
