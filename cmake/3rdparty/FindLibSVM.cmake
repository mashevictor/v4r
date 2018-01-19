if(BUILD_LIBSVM)
  set(LIBSVM_INCLUDE_DIR "${V4R_3P_LIBSVM_INSTALL_DIR}/include")
  set(LIBSVM_LIBRARY "${V4R_3P_LIBSVM_INSTALL_DIR}/lib/libsvm.so.2")
else()
  find_path(LIBSVM_INCLUDE_DIR libsvm/svm.h
            PATHS /usr/local /opt /usr $ENV{LIBSVM_ROOT} ${LIBSVM_DIR}
            DOC "The path to LibSVM header"
            CMAKE_FIND_ROOT_PATH_BOTH)
  find_library(LIBSVM_LIBRARY
               NAMES svm
               PATHS /usr/local /opt /usr $ENV{LIBSVM_ROOT} ${LIBSVM_DIR}
               DOC "The LibSVM library")
endif()

if(EXISTS ${LIBSVM_INCLUDE_DIR} AND EXISTS ${LIBSVM_LIBRARY})
  # Create imported target
  v4r_add_imported_library(libsvm
    IMPORTED_LOCATION ${LIBSVM_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${LIBSVM_INCLUDE_DIR}
  )
  # Find version number
  v4r_parse_header("${LIBSVM_INCLUDE_DIR}/libsvm/svm.h" LIBSVM_VERSION_LINES LIBSVM_VERSION)
  v4r_clear_vars(LIBSVM_VERSION_LINES)
  set(HAVE_LIBSVM TRUE)
else()
  set(HAVE_LIBSVM FALSE)
endif()

mark_as_advanced(LIBSVM_INCLUDE_DIR LIBSVM_LIBRARY)

