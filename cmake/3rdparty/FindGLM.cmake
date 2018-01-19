if(BUILD_GLM)
  set(GLM_INCLUDE_DIR ${V4R_3P_GLM_INSTALL_DIR}/include)
else()
  find_path(GLM_INCLUDE_DIR glm/glm.hpp
            PATHS /usr/local/include /usr/include $ENV{GLM_ROOT} ${GLM_DIR}
            DOC "The path to GLM headers")
endif()

if(EXISTS ${GLM_INCLUDE_DIR})
  # Create imported target
  v4r_add_imported_library(glm
    INTERFACE_INCLUDE_DIRECTORIES ${GLM_INCLUDE_DIR}
  )
  # Find version number
  v4r_parse_header("${GLM_INCLUDE_DIR}/glm/detail/setup.hpp" GLM_VERSION_LINES GLM_VERSION_MAJOR GLM_VERSION_MINOR GLM_VERSION_PATCH)
  set(GLM_VERSION "${GLM_VERSION_MAJOR}.${GLM_VERSION_MINOR}.${GLM_VERSION_PATCH}")
  v4r_clear_vars(GLM_VERSION_LINES GLM_VERSION_MAJOR GLM_VERSION_MINOR GLM_VERSION_PATCH)
  set(HAVE_GLM TRUE)
else()
  set(HAVE_GLM FALSE)
endif()

mark_as_advanced(GLM_INCLUDE_DIR)
