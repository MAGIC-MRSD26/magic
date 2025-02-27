#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pid_controller::pid_controller" for configuration "Release"
set_property(TARGET pid_controller::pid_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pid_controller::pid_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpid_controller.so"
  IMPORTED_SONAME_RELEASE "libpid_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pid_controller::pid_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_pid_controller::pid_controller "${_IMPORT_PREFIX}/lib/libpid_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
