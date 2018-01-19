# We only support building bundled version of EDT, so we know for sure where it is installed
v4r_add_imported_library(edt
  IMPORTED_LOCATION "${V4R_3P_EDT_INSTALL_DIR}/lib/libedt.a"
  INTERFACE_INCLUDE_DIRECTORIES "${V4R_3P_EDT_INSTALL_DIR}/include"
)

set(HAVE_EDT TRUE)
