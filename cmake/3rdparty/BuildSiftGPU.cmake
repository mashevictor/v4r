find_package(DevIL)
if(NOT IL_FOUND)
  message(FATAL_ERROR "SiftGPU requires DevIL")
endif()

v4r_build_external_project("SiftGPU"
  URL "https://repo.acin.tuwien.ac.at/tmp/permanent/SiftGPU-V400.zip"
  URL_HASH SHA1=eac08305f14cb35d12a22d493f84f9c21f6b7fdb
  CONFIGURE_COMMAND NO
  BUILD_COMMAND "make siftgpu_enable_cuda=0"
  BUILD_IN_SOURCE
  INSTALL_COMMAND "mkdir -p <INSTALL_DIR>/include/SiftGPU && "
                  "cp <SOURCE_DIR>/src/SiftGPU/SiftGPU.h <INSTALL_DIR>/include/SiftGPU && "
                  "mkdir -p <INSTALL_DIR>/lib && "
                  "cp <SOURCE_DIR>/bin/libsiftgpu.a <INSTALL_DIR>/lib"
)
