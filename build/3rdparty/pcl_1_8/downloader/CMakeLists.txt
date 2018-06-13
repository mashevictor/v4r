cmake_minimum_required(VERSION 2.8.12)
project(pcl_1_8)
include(ExternalProject)
ExternalProject_Add(pcl_1_8
  CMAKE_ARGS -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DPCL_DIR=/usr/local/share/pcl-1.8
  TIMEOUT 30
  TMP_DIR "/home/victor/software/v4r/build/3rdparty/pcl_1_8/tmp"
  STAMP_DIR "/home/victor/software/v4r/build/3rdparty/pcl_1_8/stamp"
  DOWNLOAD_DIR "/home/victor/software/v4r/build/3rdparty/download"
  SOURCE_DIR "/home/victor/software/v4r/3rdparty/pcl_1_8"
  BINARY_DIR ""
  INSTALL_DIR "/home/victor/software/v4r/build/3rdparty/pcl_1_8/install"
)
ExternalProject_Add_Step(pcl_1_8 ROBUST_PATCH
  COMMAND ${CMAKE_COMMAND} -DWORKING_DIR=<SOURCE_DIR> -DPATCHES_DIR=/home/victor/software/v4r/3rdparty/pcl_1_8 -P /home/victor/software/v4r/cmake/scripts/ApplyPatches.cmake
  COMMENT "Performing robust patch step"
  DEPENDEES update
)
