v4r_build_external_project("GLM"
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG "0.9.8.5"
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
)
