cmake_minimum_required(VERSION 2.8.12)
project(radical)
include(ExternalProject)
ExternalProject_Add(radical
  URL https://github.com/taketwo/radical/archive/1.0.0.tar.gz
  URL_HASH SHA256=672751353e83472ba81bdde0bfad550784dd1e66502d88721aa9b406a5a90ea8
  CMAKE_ARGS -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DBUILD_APPS=OFF -DOpenCV_DIR=/opt/ros/kinetic/share/OpenCV-3.3.1-dev
  TIMEOUT 30
  TMP_DIR "/home/victor/software/v4r/build/3rdparty/radical/tmp"
  STAMP_DIR "/home/victor/software/v4r/build/3rdparty/radical/stamp"
  DOWNLOAD_DIR "/home/victor/software/v4r/build/3rdparty/download"
  SOURCE_DIR "/home/victor/software/v4r/build/3rdparty/radical/src"
  BINARY_DIR ""
  INSTALL_DIR "/home/victor/software/v4r/build/3rdparty/radical/install"
)
ExternalProject_Add_Step(radical ROBUST_PATCH
  COMMAND ${CMAKE_COMMAND} -DWORKING_DIR=<SOURCE_DIR> -DPATCHES_DIR=/home/victor/software/v4r/3rdparty/radical -P /home/victor/software/v4r/cmake/scripts/ApplyPatches.cmake
  COMMENT "Performing robust patch step"
  DEPENDEES update
)
