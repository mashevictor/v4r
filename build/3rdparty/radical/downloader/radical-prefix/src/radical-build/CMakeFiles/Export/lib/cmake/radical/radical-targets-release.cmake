#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "radical" for configuration "Release"
set_property(TARGET radical APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(radical PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libradical.so.1.0.0"
  IMPORTED_SONAME_RELEASE "libradical.so.1.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS radical )
list(APPEND _IMPORT_CHECK_FILES_FOR_radical "${_IMPORT_PREFIX}/lib/libradical.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
