v4r_clear_vars(CAFFE_INCLUDE_DIRS CAFFE_LIBRARIES CAFFE_VERSION)

find_package(Caffe QUIET)
if(Caffe_FOUND)
  get_target_property(CAFFE_INCLUDE_DIRS caffe INTERFACE_INCLUDE_DIRECTORIES)
  set(CAFFE_LIBRARIES "${Caffe_LIBRARIES}")
  set(HAVE_CAFFE TRUE)
else()
  set(HAVE_CAFFE FALSE)
endif()
