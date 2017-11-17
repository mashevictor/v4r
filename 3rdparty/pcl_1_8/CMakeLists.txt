cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_1_8)

# This project contains files from PCL which are either not available in or have a different ABI
# than the currently released PCL version.

find_package(PCL REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})

set(SOURCES
  uniform_sampling.cpp
  conditional_euclidean_clustering.cpp
  organized_edge_detection.cpp
  convolution_2d.cpp
)

set(HEADERS
  pcl_1_8/keypoints/impl/uniform_sampling.hpp
  pcl_1_8/segmentation/impl/conditional_euclidean_clustering.hpp
  pcl_1_8/features/impl/organized_edge_detection.hpp
  pcl_1_8/2d/impl/convolution.hpp
  pcl_1_8/2d/impl/kernel.hpp
  pcl_1_8/2d/impl/edge.hpp
  pcl_1_8/keypoints/uniform_sampling.h
  pcl_1_8/segmentation/conditional_euclidean_clustering.h
  pcl_1_8/features/organized_edge_detection.h
  pcl_1_8/2d/edge.h
  pcl_1_8/2d/convolution.h
  pcl_1_8/2d/kernel.h
)

add_library(pcl_1_8 STATIC ${SOURCES})
target_link_libraries(pcl_1_8 ${PCL_LIBRARIES})

set_target_properties(pcl_1_8 PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

install(DIRECTORY pcl_1_8 DESTINATION "include")
install(TARGETS pcl_1_8 ARCHIVE DESTINATION lib)
