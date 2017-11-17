# We only support building bundled version of EDT, so we know for sure where it is installed
set(EDT_INCLUDE_DIRS ${V4R_3P_EDT_INSTALL_DIR}/include)
set(EDT_LIBRARIES ${V4R_3P_EDT_INSTALL_DIR}/lib/libedt.a)
set(HAVE_EDT TRUE)
