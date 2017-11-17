v4r_clear_vars(RADICAL_INCLUDE_DIRS RADICAL_LIBRARIES RADICAL_VERSION)

if(BUILD_RADICAL)
  set(radical_DIR ${V4R_3P_RADICAL_INSTALL_DIR}/lib/cmake/radical)
endif()

find_package(radical CONFIG)

if(radical_FOUND)
  set(RADICAL_LIBRARIES radical)
  set(RADICAL_VERSION "${radical_VERSION}")
  get_target_property(RADICAL_INCLUDE_DIRS radical INTERFACE_INCLUDE_DIRECTORIES)
  set(HAVE_RADICAL TRUE)
else()
  set(HAVE_RADICAL FALSE)
endif()
