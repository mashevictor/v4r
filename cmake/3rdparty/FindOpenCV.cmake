find_package(OpenCV)
if(OpenCV_FOUND)
  set(OPENCV_VERSION "${OpenCV_VERSION}")
  v4r_add_imported_library(opencv
    INTERFACE_LINK_LIBRARIES ${OpenCV_LIBS}
  )
  # This variable may be used to configure third-party dependencies to use this same OpenCV library
  set(OPENCV_CONFIG_PATH "${OpenCV_CONFIG_PATH}" CACHE INTERNAL "Directory with OpenCV CMake configuration files")
  set(HAVE_OPENCV TRUE)
else()
  set(HAVE_OPENCV FALSE)
endif()

