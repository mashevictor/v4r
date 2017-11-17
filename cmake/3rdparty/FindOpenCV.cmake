find_package(OpenCV)
if(OpenCV_FOUND)
  set(OPENCV_LIBRARIES "${OpenCV_LIBS}")
  set(OPENCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}")
  set(OPENCV_VERSION "${OpenCV_VERSION}")
  set(OPENCV_ROOT "${OpenCV_INSTALL_PATH}")
  # This variable may be used to configure third-party dependencies to use this same OpenCV library
  set(OPENCV_CONFIG_PATH "${OpenCV_CONFIG_PATH}" CACHE INTERNAL "Directory with OpenCV CMake configuration files")
  set(HAVE_OPENCV TRUE)
else()
  set(HAVE_OPENCV FALSE)
endif()

