cmake_minimum_required(VERSION 2.8.12)
project(metslib)
include(ExternalProject)
ExternalProject_Add(metslib
  CMAKE_ARGS -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
  TIMEOUT 30
  TMP_DIR "/home/victor/software/v4r/build/3rdparty/metslib/tmp"
  STAMP_DIR "/home/victor/software/v4r/build/3rdparty/metslib/stamp"
  DOWNLOAD_DIR "/home/victor/software/v4r/build/3rdparty/download"
  SOURCE_DIR "/home/victor/software/v4r/3rdparty/metslib"
  BINARY_DIR ""
  INSTALL_DIR "/home/victor/software/v4r/build/3rdparty/metslib/install"
)
ExternalProject_Add_Step(metslib ROBUST_PATCH
  COMMAND ${CMAKE_COMMAND} -DWORKING_DIR=<SOURCE_DIR> -DPATCHES_DIR=/home/victor/software/v4r/3rdparty/metslib -P /home/victor/software/v4r/cmake/scripts/ApplyPatches.cmake
  COMMENT "Performing robust patch step"
  DEPENDEES update
)
