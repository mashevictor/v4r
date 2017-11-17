# We only support building bundled version of EDT, so we know for sure where it is installed
set(METSLIB_INCLUDE_DIRS ${V4R_3P_METSLIB_INSTALL_DIR}/include)
set(METSLIB_LIBRARIES "")
set(HAVE_METSLIB TRUE)
