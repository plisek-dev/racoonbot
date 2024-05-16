#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "racoonbot_pkg::racoonbot_pkg" for configuration ""
set_property(TARGET racoonbot_pkg::racoonbot_pkg APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(racoonbot_pkg::racoonbot_pkg PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libracoonbot_pkg.so"
  IMPORTED_SONAME_NOCONFIG "libracoonbot_pkg.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS racoonbot_pkg::racoonbot_pkg )
list(APPEND _IMPORT_CHECK_FILES_FOR_racoonbot_pkg::racoonbot_pkg "${_IMPORT_PREFIX}/lib/libracoonbot_pkg.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
