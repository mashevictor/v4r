# Install script for directory: /home/victor/software/v4r/modules/keypoints

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libs")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so.2.0.6"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so.2.0"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY OPTIONAL FILES
    "/home/victor/software/v4r/build/lib/libv4r_keypoints.so.2.0.6"
    "/home/victor/software/v4r/build/lib/libv4r_keypoints.so.2.0"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so.2.0.6"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so.2.0"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/victor/software/v4r/build/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib:"
           NEW_RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so"
         RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/victor/software/v4r/build/lib/libv4r_keypoints.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so"
         OLD_RPATH "/home/victor/software/v4r/build/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib:"
         NEW_RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_keypoints.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/all_headers.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/RigidTransformationRANSAC.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/CodebookMatcher.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/keypoint_extractor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/types.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/PlaneEstimationRANSAC.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/temporal_smoothing_filter.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/uniform_sampling_extractor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/Part.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/PartRotation1D.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/narf_keypoint_extractor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/ArticulatedObject.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/PartMotion6D.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/harris3d_keypoint_extractor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/ClusterNormalsToPlanes.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/io.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/iss_keypoint_extractor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/warpPatchHomography.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/Object.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/opencv_serialization.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/toString.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/pair_serialization.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/PoseIO.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/invPose.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/eigen_boost_serialization.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/triple_serialization.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/triple.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/keypoints/impl" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/keypoints/include/v4r/keypoints/impl/ArticulatedObject_serialization.hpp")
endif()

