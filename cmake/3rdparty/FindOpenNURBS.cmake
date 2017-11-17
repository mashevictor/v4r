# We only support building bundled version of OpenNURBS, so we know for sure where it is installed
set(OPENNURBS_INCLUDE_DIRS ${V4R_3P_OPENNURBS_INSTALL_DIR}/include)
set(OPENNURBS_LIBRARIES ${V4R_3P_OPENNURBS_INSTALL_DIR}/lib/libopennurbs.a)
set(HAVE_OPENNURBS TRUE)

