v4r_clear_vars(LIBSVM_INCLUDE_DIRS LIBSVM_LIBRARIES LIBSVM_VERSION)

if(BUILD_LIBSVM)
  set(LIBSVM_INCLUDE_DIRS ${V4R_3P_LIBSVM_INSTALL_DIR}/include)
  set(LIBSVM_LIBRARIES ${V4R_3P_LIBSVM_INSTALL_DIR}/lib/libsvm.so.2)
else()
  find_path(LIBSVM_INCLUDE_DIRS libsvm/svm.h
            PATHS /usr/local /opt /usr $ENV{LIBSVM_ROOT} ${LIBSVM_DIR}
            DOC "The path to LibSVM header"
            CMAKE_FIND_ROOT_PATH_BOTH)
  find_library(LIBSVM_LIBRARIES
               NAMES svm
               PATHS /usr/local /opt /usr $ENV{LIBSVM_ROOT} ${LIBSVM_DIR}
               DOC "The LibSVM library")
endif()

if(EXISTS ${LIBSVM_INCLUDE_DIRS} AND EXISTS ${LIBSVM_LIBRARIES})
  v4r_parse_header("${LIBSVM_INCLUDE_DIRS}/libsvm/svm.h" LIBSVM_VERSION_LINES LIBSVM_VERSION)
  v4r_clear_vars(LIBSVM_VERSION_LINES)
  set(HAVE_LIBSVM TRUE)
else()
  set(HAVE_LIBSVM FALSE)
endif()

