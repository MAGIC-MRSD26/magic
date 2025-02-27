#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "steering_controllers_library::steering_controllers_library" for configuration "Release"
set_property(TARGET steering_controllers_library::steering_controllers_library APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(steering_controllers_library::steering_controllers_library PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsteering_controllers_library.so"
  IMPORTED_SONAME_RELEASE "libsteering_controllers_library.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS steering_controllers_library::steering_controllers_library )
list(APPEND _IMPORT_CHECK_FILES_FOR_steering_controllers_library::steering_controllers_library "${_IMPORT_PREFIX}/lib/libsteering_controllers_library.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
