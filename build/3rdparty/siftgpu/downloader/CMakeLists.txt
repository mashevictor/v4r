cmake_minimum_required(VERSION 2.8.12)
project(siftgpu)
include(ExternalProject)
ExternalProject_Add(siftgpu
  URL /home/victor/SiftGPU-master
  URL_HASH SHA1=a1c361e34a05dbfc4acc42f9004ba8ac
  CONFIGURE_COMMAND ""
  BUILD_COMMAND make siftgpu_enable_cuda=0
  BUILD_IN_SOURCE INSTALL_COMMAND
  mkdir -p <INSTALL_DIR>/include/SiftGPU &&  cp <SOURCE_DIR>/src/SiftGPU/SiftGPU.h <INSTALL_DIR>/include/SiftGPU && 
  mkdir -p <INSTALL_DIR>/lib &&  cp <SOURCE_DIR>/bin/libsiftgpu.a <INSTALL_DIR>/lib
  TIMEOUT 30
  TMP_DIR "/home/victor/software/v4r/build/3rdparty/siftgpu/tmp"
  STAMP_DIR "/home/victor/software/v4r/build/3rdparty/siftgpu/stamp"
  DOWNLOAD_DIR "/home/victor/software/v4r/build/3rdparty/download"
  SOURCE_DIR "/home/victor/software/v4r/build/3rdparty/siftgpu/src"
  BINARY_DIR "/home/victor/software/v4r/build/3rdparty/siftgpu/src"
  INSTALL_DIR "/home/victor/software/v4r/build/3rdparty/siftgpu/install"
)
ExternalProject_Add_Step(siftgpu ROBUST_PATCH
  COMMAND ${CMAKE_COMMAND} -DWORKING_DIR=<SOURCE_DIR> -DPATCHES_DIR=/home/victor/software/v4r/3rdparty/siftgpu -P /home/victor/software/v4r/cmake/scripts/ApplyPatches.cmake
  COMMENT "Performing robust patch step"
  DEPENDEES update
)
