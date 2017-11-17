find_package(GLEW)
if(GLEW_FOUND)
  set(HAVE_GLEW TRUE)
  set(GLEW_INCLUDE_DIRS "${GLEW_INCLUDE_DIR}")
else()
  set(HAVE_GLEW FALSE)
endif()

