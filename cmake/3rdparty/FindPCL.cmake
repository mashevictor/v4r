find_package(PCL 1.7.1 QUIET)
if(PCL_FOUND)
  v4r_list_filterout(PCL_LIBRARIES "optimized|debug|vtkproj4")
  list(REMOVE_DUPLICATES PCL_LIBRARIES)
  v4r_add_imported_library(pcl
    INTERFACE_INCLUDE_DIRECTORIES ${PCL_INCLUDE_DIRS}
    INTERFACE_LINK_LIBRARIES ${PCL_LIBRARIES}
  )
  # This variable may be used to configure third-party dependencies to use this same PCL library
  set(PCL_CONFIG_PATH ${PCL_ROOT}/share/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR} CACHE INTERNAL "Directory with PCL CMake configuration files")
  set(HAVE_PCL TRUE)
else()
  set(HAVE_PCL FALSE)
endif()

