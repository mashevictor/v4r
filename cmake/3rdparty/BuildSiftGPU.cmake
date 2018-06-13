find_package(DevIL)
if(NOT IL_FOUND)
  message(FATAL_ERROR "SiftGPU requires DevIL")
endif()

v4r_build_external_project("SiftGPU"
  URL "/home/victor/SiftGPU-master"
  URL_HASH SHA1=a1c361e34a05dbfc4acc42f9004ba8ac
  CONFIGURE_COMMAND NO
  BUILD_COMMAND "make siftgpu_enable_cuda=0"
  BUILD_IN_SOURCE
  INSTALL_COMMAND "mkdir -p <INSTALL_DIR>/include/SiftGPU && "
                  "cp <SOURCE_DIR>/src/SiftGPU/SiftGPU.h <INSTALL_DIR>/include/SiftGPU && "
                  "mkdir -p <INSTALL_DIR>/lib && "
                  "cp <SOURCE_DIR>/bin/libsiftgpu.a <INSTALL_DIR>/lib"
)
