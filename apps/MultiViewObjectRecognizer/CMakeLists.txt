SET(OR_DEPS v4r_core v4r_common v4r_io v4r_features v4r_keypoints v4r_segmentation v4r_recognition v4r_ml)
v4r_check_dependencies(${OR_DEPS})

if(NOT V4R_DEPENDENCIES_FOUND)
  message(***MVObjectRecognizer does not meet dependencies*****)
  return()
endif()

v4r_include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
v4r_include_modules(${OR_DEPS})

set(incs visualization.h)
set(srcs visualization.cpp main.cpp)

add_executable(MVObjectRecognizer ${incs} ${srcs})
target_link_libraries(MVObjectRecognizer ${OR_DEPS} ${DEP_LIBS})

INSTALL(TARGETS MVObjectRecognizer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
