# We only support building bundled version of METSlib, so we know for sure where it is installed
v4r_add_imported_library(metslib
  INTERFACE_INCLUDE_DIRECTORIES "${V4R_3P_METSLIB_INSTALL_DIR}/include"
)

set(HAVE_METSLIB TRUE)
