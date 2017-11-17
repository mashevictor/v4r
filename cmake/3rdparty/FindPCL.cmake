find_package(PCL "${MIN_VER_PCL}")
if(PCL_FOUND)
  set(HAVE_PCL TRUE)
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
  # This variable may be used to configure third-party dependencies to use this same PCL library
  set(PCL_CONFIG_PATH ${PCL_ROOT}/share/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR} CACHE INTERNAL "Directory with PCL CMake configuration files")
else()
  set(HAVE_PCL FALSE)
endif()

