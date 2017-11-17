cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(OpenNURBS)

include("openNURBS.cmake")
include("on_nurbs.cmake")

find_package(PCL REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})

add_library(opennurbs STATIC ${OPENNURBS_SOURCES} ${ON_NURBS_SOURCES})
target_link_libraries(opennurbs ${PCL_LIBRARIES})

set_target_properties(opennurbs PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

install(FILES ${OPENNURBS_INCLUDES} DESTINATION "include/opennurbs")
install(TARGETS opennurbs ARCHIVE DESTINATION lib)
