#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ackermann_steering_controller::ackermann_steering_controller" for configuration "Release"
set_property(TARGET ackermann_steering_controller::ackermann_steering_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ackermann_steering_controller::ackermann_steering_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libackermann_steering_controller.so"
  IMPORTED_SONAME_RELEASE "libackermann_steering_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ackermann_steering_controller::ackermann_steering_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_ackermann_steering_controller::ackermann_steering_controller "${_IMPORT_PREFIX}/lib/libackermann_steering_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
