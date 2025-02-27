#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "force_torque_sensor_broadcaster::force_torque_sensor_broadcaster" for configuration "Release"
set_property(TARGET force_torque_sensor_broadcaster::force_torque_sensor_broadcaster APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(force_torque_sensor_broadcaster::force_torque_sensor_broadcaster PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libforce_torque_sensor_broadcaster.so"
  IMPORTED_SONAME_RELEASE "libforce_torque_sensor_broadcaster.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS force_torque_sensor_broadcaster::force_torque_sensor_broadcaster )
list(APPEND _IMPORT_CHECK_FILES_FOR_force_torque_sensor_broadcaster::force_torque_sensor_broadcaster "${_IMPORT_PREFIX}/lib/libforce_torque_sensor_broadcaster.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
