#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pose_broadcaster::pose_broadcaster" for configuration "Release"
set_property(TARGET pose_broadcaster::pose_broadcaster APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pose_broadcaster::pose_broadcaster PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpose_broadcaster.so"
  IMPORTED_SONAME_RELEASE "libpose_broadcaster.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pose_broadcaster::pose_broadcaster )
list(APPEND _IMPORT_CHECK_FILES_FOR_pose_broadcaster::pose_broadcaster "${_IMPORT_PREFIX}/lib/libpose_broadcaster.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
