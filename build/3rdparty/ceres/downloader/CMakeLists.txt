cmake_minimum_required(VERSION 2.8.12)
project(ceres)
include(ExternalProject)
ExternalProject_Add(ceres
  URL http://ceres-solver.org/ceres-solver-1.13.0.tar.gz
  URL_HASH SHA256=1DF490A197634D3AAB0A65687DECD362912869C85A61090FF66F073C967A7DCD
  CMAKE_ARGS -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DGLOG_INCLUDE_DIR=/usr/include -DGLOG_LIBRARY=/usr/lib/x86_64-linux-gnu/libglog.so
  TIMEOUT 30
  TMP_DIR "/home/victor/software/v4r/build/3rdparty/ceres/tmp"
  STAMP_DIR "/home/victor/software/v4r/build/3rdparty/ceres/stamp"
  DOWNLOAD_DIR "/home/victor/software/v4r/build/3rdparty/download"
  SOURCE_DIR "/home/victor/software/v4r/build/3rdparty/ceres/src"
  BINARY_DIR ""
  INSTALL_DIR "/home/victor/software/v4r/build/3rdparty/ceres/install"
)
ExternalProject_Add_Step(ceres ROBUST_PATCH
  COMMAND ${CMAKE_COMMAND} -DWORKING_DIR=<SOURCE_DIR> -DPATCHES_DIR=/home/victor/software/v4r/3rdparty/ceres -P /home/victor/software/v4r/cmake/scripts/ApplyPatches.cmake
  COMMENT "Performing robust patch step"
  DEPENDEES update
)
