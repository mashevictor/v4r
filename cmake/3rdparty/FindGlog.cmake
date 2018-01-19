include(FindPkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_search_module(GLOG QUIET libglog)
  if(GLOG_FOUND)
    v4r_add_imported_library(glog
      IMPORTED_LOCATION "${GLOG_LIBDIR}/libglog.so"
      INTERFACE_INCLUDE_DIRECTORIES ${GLOG_INCLUDEDIR}
    )
    set(HAVE_GLOG TRUE)
  else()
    set(HAVE_GLOG FALSE)
  endif()
  # Completely wipe this variable, because other 3rd party libraries may want to discover Glog themselves (e.g. Ceres)
  v4r_clear_vars(GLOG_FOUND)
endif()
