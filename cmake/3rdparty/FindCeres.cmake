v4r_clear_vars(CERES_INCLUDE_DIRS CERES_LIBRARIES CERES_VERSION)

if(BUILD_CERES)
  set(Ceres_DIR ${V4R_3P_CERES_INSTALL_DIR}/lib/cmake/Ceres)
  find_package(Ceres 1.12.0 CONFIG)
else()
  find_package(Ceres 1.12.0)
endif()

if(CERES_FOUND)
  set(HAVE_CERES TRUE)
else()
  set(HAVE_CERES FALSE)
endif()
