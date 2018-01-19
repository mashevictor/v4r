find_package(Caffe QUIET)

if(Caffe_FOUND)
  set(HAVE_CAFFE TRUE)
else()
  set(HAVE_CAFFE FALSE)
endif()

mark_as_advanced(Caffe_DIR)
