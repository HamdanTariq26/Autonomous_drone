# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target tello_autonomy_msgs::tello_autonomy_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${tello_autonomy_msgs_TARGETS}.
if(tello_autonomy_msgs_TARGETS AND NOT TARGET tello_autonomy_msgs::tello_autonomy_msgs)
  add_library(tello_autonomy_msgs::tello_autonomy_msgs INTERFACE IMPORTED)
  set_target_properties(tello_autonomy_msgs::tello_autonomy_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${tello_autonomy_msgs_TARGETS}")
endif()
