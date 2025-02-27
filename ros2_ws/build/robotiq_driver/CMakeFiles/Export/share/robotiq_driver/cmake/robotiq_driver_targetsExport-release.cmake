#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robotiq_driver::robotiq_driver" for configuration "Release"
set_property(TARGET robotiq_driver::robotiq_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robotiq_driver::robotiq_driver PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobotiq_driver.so"
  IMPORTED_SONAME_RELEASE "librobotiq_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS robotiq_driver::robotiq_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_robotiq_driver::robotiq_driver "${_IMPORT_PREFIX}/lib/librobotiq_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
