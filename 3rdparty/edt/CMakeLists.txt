cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(EDT)

find_package(PCL REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})

set(lib_srcs
  propagation_distance_field.cpp
)

SET(lib_hdrs
  distance_field.h
  voxel_grid.h
  propagation_distance_field.h
)

add_library(edt STATIC ${lib_srcs})
target_link_libraries(edt ${PCL_LIBRARIES})

set_target_properties(edt PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

install(FILES ${lib_hdrs} DESTINATION "include/EDT")
install(TARGETS edt ARCHIVE DESTINATION lib)
