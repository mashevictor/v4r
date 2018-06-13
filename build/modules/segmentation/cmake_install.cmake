# Install script for directory: /home/victor/software/v4r/modules/segmentation

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
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so.2.0.6"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so.2.0"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY OPTIONAL FILES
    "/home/victor/software/v4r/build/lib/libv4r_segmentation.so.2.0.6"
    "/home/victor/software/v4r/build/lib/libv4r_segmentation.so.2.0"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so.2.0.6"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so.2.0"
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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so"
         RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/victor/software/v4r/build/lib/libv4r_segmentation.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so"
         OLD_RPATH "/home/victor/software/v4r/build/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib:"
         NEW_RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_segmentation.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/all_headers.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/plane_extractor_sac.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/plane_extractor_organized_multiplane.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/ClusterNormalsToPlanesPCL.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/segmenter_2d_connected_components.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/segmenter.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/segmenter_organized_connected_component.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/SlicRGBD.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/types.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/segmentation_utils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/smooth_Euclidean_segmenter.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/Slic.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/plane_extractor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/plane_extractor_sac_normals.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/segmenter_euclidean.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/plane_extractor_tile.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/SLICO.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/plane_utils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/segmentation/include/v4r/segmentation/ClusterNormalsToPlanes.h")
endif()

