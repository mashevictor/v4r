find_package(OpenGL)
if(OPENGL_FOUND)
  v4r_add_imported_library(opengl
    INTERFACE_INCLUDE_DIRECTORIES ${OPENGL_INCLUDE_DIR}
    INTERFACE_LINK_LIBRARIES GL
  )
  set(HAVE_OPENGL TRUE)
else()
  set(HAVE_OPENGL FALSE)
endif()
