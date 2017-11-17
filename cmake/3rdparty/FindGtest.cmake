# We only support building Gtest from source, so we know for sure where it is installed
set(GTEST_INCLUDE_DIRS ${V4R_3P_GTEST_INSTALL_DIR}/include)
set(GTEST_LIBRARIES gtest)
set(GTEST_VERSION "1.8.0")
set(HAVE_GTEST TRUE)

add_library(gtest STATIC IMPORTED GLOBAL)
set_target_properties(gtest PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}"
  IMPORTED_LOCATION_RELEASE "${V4R_3P_GTEST_INSTALL_DIR}/lib/libgtest.a"
)
