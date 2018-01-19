# We only support building bundled version of OpenNURBS, so we know for sure where it is installed
v4r_add_imported_library(opennurbs
  IMPORTED_LOCATION "${V4R_3P_OPENNURBS_INSTALL_DIR}/lib/libopennurbs.a"
  INTERFACE_INCLUDE_DIRECTORIES "${V4R_3P_OPENNURBS_INSTALL_DIR}/include"
)

set(HAVE_OPENNURBS TRUE)
