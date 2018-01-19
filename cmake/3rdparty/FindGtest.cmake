# We only support building Gtest from source, so we know for sure where it is installed
v4r_add_imported_library(gtest
  IMPORTED_LOCATION "${V4R_3P_GTEST_INSTALL_DIR}/lib/libgtest.a"
  INTERFACE_INCLUDE_DIRECTORIES "${V4R_3P_GTEST_INSTALL_DIR}/include"
)

set(GTEST_VERSION "1.8.0")
set(HAVE_GTEST TRUE)
