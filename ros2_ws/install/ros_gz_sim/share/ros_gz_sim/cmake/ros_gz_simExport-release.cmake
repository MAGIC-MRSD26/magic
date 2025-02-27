#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros_gz_sim::ros_gz_sim" for configuration "Release"
set_property(TARGET ros_gz_sim::ros_gz_sim APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ros_gz_sim::ros_gz_sim PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libros_gz_sim.so"
  IMPORTED_SONAME_RELEASE "libros_gz_sim.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ros_gz_sim::ros_gz_sim )
list(APPEND _IMPORT_CHECK_FILES_FOR_ros_gz_sim::ros_gz_sim "${_IMPORT_PREFIX}/lib/libros_gz_sim.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
