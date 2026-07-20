# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_octomap_manager_shim_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED octomap_manager_shim_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(octomap_manager_shim_FOUND FALSE)
  elseif(NOT octomap_manager_shim_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(octomap_manager_shim_FOUND FALSE)
  endif()
  return()
endif()
set(_octomap_manager_shim_CONFIG_INCLUDED TRUE)

# output package information
if(NOT octomap_manager_shim_FIND_QUIETLY)
  message(STATUS "Found octomap_manager_shim: 0.0.1 (${octomap_manager_shim_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'octomap_manager_shim' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${octomap_manager_shim_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(octomap_manager_shim_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${octomap_manager_shim_DIR}/${_extra}")
endforeach()
