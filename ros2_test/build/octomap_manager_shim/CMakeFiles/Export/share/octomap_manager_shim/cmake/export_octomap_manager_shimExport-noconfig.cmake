#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "octomap_manager_shim::octomap_manager_shim" for configuration ""
set_property(TARGET octomap_manager_shim::octomap_manager_shim APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(octomap_manager_shim::octomap_manager_shim PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liboctomap_manager_shim.so"
  IMPORTED_SONAME_NOCONFIG "liboctomap_manager_shim.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS octomap_manager_shim::octomap_manager_shim )
list(APPEND _IMPORT_CHECK_FILES_FOR_octomap_manager_shim::octomap_manager_shim "${_IMPORT_PREFIX}/lib/liboctomap_manager_shim.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
