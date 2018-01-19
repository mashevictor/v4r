find_path(EIGEN_INCLUDE_DIR "Eigen/Core"
          PATHS /usr/local /opt /usr $ENV{EIGEN_ROOT}/include ${EIGEN_DIR}
          PATH_SUFFIXES include/eigen3 Eigen/include/eigen3
          DOC "The path to Eigen3 headers"
          CMAKE_FIND_ROOT_PATH_BOTH)

if(EXISTS ${EIGEN_INCLUDE_DIR})
  # Create imported target
  v4r_add_imported_library(eigen
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN_INCLUDE_DIR}
  )
  # Find version number in headers
  v4r_parse_header("${EIGEN_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" EIGEN_VERSION_LINES EIGEN_WORLD_VERSION EIGEN_MAJOR_VERSION EIGEN_MINOR_VERSION)
  set(EIGEN_VERSION "${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION}")
  v4r_clear_vars(EIGEN_VERSION_LINES EIGEN_WORLD_VERSION EIGEN_MAJOR_VERSION EIGEN_MINOR_VERSION)
  set(HAVE_EIGEN TRUE)
else()
  set(HAVE_EIGEN FALSE)
endif()

mark_as_advanced(EIGEN_INCLUDE_DIR)
