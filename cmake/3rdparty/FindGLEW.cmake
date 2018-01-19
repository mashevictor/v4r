find_package(GLEW)
if(GLEW_FOUND)
  set(HAVE_GLEW TRUE)
  v4r_add_imported_library(glew
    IMPORTED_LOCATION ${GLEW_LIBRARIES}
    INTERFACE_INCLUDE_DIRECTORIES ${GLEW_INCLUDE_DIR}
  )
else()
  set(HAVE_GLEW FALSE)
endif()

