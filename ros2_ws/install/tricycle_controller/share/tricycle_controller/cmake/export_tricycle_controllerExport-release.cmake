#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "tricycle_controller::tricycle_controller" for configuration "Release"
set_property(TARGET tricycle_controller::tricycle_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(tricycle_controller::tricycle_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtricycle_controller.so"
  IMPORTED_SONAME_RELEASE "libtricycle_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS tricycle_controller::tricycle_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_tricycle_controller::tricycle_controller "${_IMPORT_PREFIX}/lib/libtricycle_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
