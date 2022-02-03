#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "RobofleetBase::robofleet_base" for configuration ""
set_property(TARGET RobofleetBase::robofleet_base APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(RobofleetBase::robofleet_base PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librobofleet_base.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS RobofleetBase::robofleet_base )
list(APPEND _IMPORT_CHECK_FILES_FOR_RobofleetBase::robofleet_base "${_IMPORT_PREFIX}/lib/librobofleet_base.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
