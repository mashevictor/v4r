# Install script for directory: /home/victor/software/v4r/modules/attention_segmentation

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
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so.2.0.6"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so.2.0"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY OPTIONAL FILES
    "/home/victor/software/v4r/build/lib/libv4r_attention_segmentation.so.2.0.6"
    "/home/victor/software/v4r/build/lib/libv4r_attention_segmentation.so.2.0"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so.2.0.6"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so.2.0"
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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so"
         RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/victor/software/v4r/build/lib/libv4r_attention_segmentation.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so"
         OLD_RPATH "/home/victor/software/v4r/build/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib:"
         NEW_RPATH "/usr/local/lib:/usr/local/share/V4R/3rdparty/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4r_attention_segmentation.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/algo.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/segmentation.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/normalization.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/PCLCommonHeaders.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/LocationMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/Relation.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/BaseMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/nurbs_tools.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/FrintropSaliencyMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/math.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/HitRatio.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/Symmetry3DMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/sphereHistogram.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/PCLUtils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/BoundaryRelationsMeanColor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/drawUtils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SymmetryMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/pyramidBase.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/BoundaryRelationsMeanCurvature.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/segmentEvaluation.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SurfaceModel.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/Graph.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/StructuralRelations.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/headers.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/BoundaryRelationsBase.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/sequential_fitter.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/EPBase.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/AddGroundTruth.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/disjoint-set.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/ep_segmentation.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/IKNSaliencyMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/AttentionModule.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/EPEvaluation.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/sparse_mat.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/RelativeSurfaceOrientationMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/pyramidFrintrop.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/WTA.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/svm.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/MSR.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SurfaceCurvatureMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/eputils_headers.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/MapsCombination.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/BoundaryRelationsMeanDepth.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/AmHitRatio.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/AttentionExampleUtils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/ZAdaptiveNormals.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SurfaceModeling.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/TJ.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/pyramidItti.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/GraphCut.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/OrientationMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SVMFileCreator.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/Fourier.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/fitting_surface_pdm.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SurfaceHeightMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/PCA.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/debugUtils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/Vs3ArcRelations.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/EPUtils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/nurbs_solve.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SVMPredictorSingle.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/timeUtils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/Edge.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/AssemblyRelations.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SVMScale.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/convertions.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/pyramidSimple.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/PPlane.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/ColorHistogram.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/ColorMap.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/cvgabor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/ClusterNormalsToPlanes.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/AttentionModuleErrors.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/connectedComponents.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/Gabor.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/utils.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/nurbs_data.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/Texture.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/SVMTrainModel.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/attention_segmentation" TYPE FILE OPTIONAL FILES "/home/victor/software/v4r/modules/attention_segmentation/include/v4r/attention_segmentation/PCLPreprocessingXYZRC.h")
endif()

