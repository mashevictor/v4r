# We only support building bundled version of PCL_1_8, so we know for sure where it is installed
v4r_add_imported_library(pcl_1_8
  INTERFACE_INCLUDE_DIRECTORIES "${V4R_3P_PCL_1_8_INSTALL_DIR}/include"
  IMPORTED_LOCATION "${V4R_3P_PCL_1_8_INSTALL_DIR}/lib/libpcl_1_8.a"
  INTERFACE_LINK_LIBRARIES pcl
)

set(HAVE_PCL_1_8 TRUE)
