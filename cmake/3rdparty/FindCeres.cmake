if(BUILD_CERES)
  set(Ceres_DIR ${V4R_3P_CERES_INSTALL_DIR}/lib/cmake/Ceres)
  find_package(Ceres 1.12.0 CONFIG)
else()
  find_package(Ceres 1.12.0)
endif()

if(CERES_FOUND)
  # For some reason Ceres (version 1.13) does not set include directories on the imported target
  set_property(TARGET ceres APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CERES_INCLUDE_DIRS})
  set(HAVE_CERES TRUE)
else()
  set(HAVE_CERES FALSE)
endif()

mark_as_advanced(Ceres_DIR)
