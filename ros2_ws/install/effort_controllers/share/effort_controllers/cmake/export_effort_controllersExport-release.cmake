#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "effort_controllers::effort_controllers" for configuration "Release"
set_property(TARGET effort_controllers::effort_controllers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(effort_controllers::effort_controllers PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libeffort_controllers.so"
  IMPORTED_SONAME_RELEASE "libeffort_controllers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS effort_controllers::effort_controllers )
list(APPEND _IMPORT_CHECK_FILES_FOR_effort_controllers::effort_controllers "${_IMPORT_PREFIX}/lib/libeffort_controllers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
