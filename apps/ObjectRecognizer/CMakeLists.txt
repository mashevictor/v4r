SET(OR_DEPS v4r_core v4r_common v4r_io v4r_features v4r_keypoints v4r_segmentation v4r_recognition v4r_registration v4r_ml v4r_apps)
v4r_check_dependencies(${OR_DEPS})

if(NOT V4R_DEPENDENCIES_FOUND)
  message(***ObjectRecognizer does not meet dependencies*****)
  return()
endif()

v4r_include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
v4r_include_modules(${OR_DEPS})

#add_definitions(-std=c++0x)
add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
add_definitions(-Dlinux -D__x86_64__)

add_executable(ObjectRecognizer main.cpp)
target_link_libraries(ObjectRecognizer ${OR_DEPS} ${DEP_LIBS} opengl)
add_executable(MVObjectRecognizerEval mv_eval.cpp)
target_link_libraries(MVObjectRecognizerEval ${OR_DEPS} ${DEP_LIBS})
add_executable(compute_recognition_rate_over_occlusion compute_recognition_rate_over_occlusion.cpp)
target_link_libraries(compute_recognition_rate_over_occlusion ${OR_DEPS} ${DEP_LIBS})
add_executable(check_hypotheses check_hypotheses.cpp)
target_link_libraries(check_hypotheses ${OR_DEPS} ${DEP_LIBS})
add_executable(compute_recognition_rate compute_recognition_rate.cpp)
target_link_libraries(compute_recognition_rate ${OR_DEPS} ${DEP_LIBS})

INSTALL(TARGETS ObjectRecognizer MVObjectRecognizerEval compute_recognition_rate_over_occlusion compute_recognition_rate
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(DIRECTORY cfg
DESTINATION share/V4R/apps/ObjectRecognizer/
)
