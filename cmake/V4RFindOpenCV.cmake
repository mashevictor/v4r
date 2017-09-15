if(WITH_OPENCV)
  if(MIN_VER_OpenCV)
    find_package(OpenCV "${MIN_VER_OpenCV}")
  else()
    find_package(OpenCV)
  endif()
  if(OpenCV_FOUND)
    set(OPENCV_LIBRARIES "${OpenCV_LIBS}")
    set(OPENCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}")
    set(HAVE_OPENCV TRUE)
  endif()
endif()