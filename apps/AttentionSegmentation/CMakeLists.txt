SET(AttentionSegmentation_DEPS v4r_core v4r_common v4r_attention_segmentation )
v4r_check_dependencies(${AttentionSegmentation_DEPS})

if(NOT V4R_DEPENDENCIES_FOUND)
  message(***AttentionSegmentation does not meet dependencies*****)
  return()
endif()

v4r_include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
v4r_include_modules(${AttentionSegmentation_DEPS})

#add_definitions(-std=c++0x)
add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
add_definitions(-Dlinux -D__x86_64__)

set(FORMS
        mainwindow.ui
        params.ui
)

SET(BINARY_NAME createTrainingSet)
add_executable(${BINARY_NAME} createTrainingSet.cpp)
target_link_libraries(${BINARY_NAME} ${AttentionSegmentation_DEPS} ${DEP_LIBS}) 

SET(BINARY_NAME scaleTrainingSet)
add_executable(${BINARY_NAME} scaleTrainingSet.cpp)
target_link_libraries(${BINARY_NAME} ${AttentionSegmentation_DEPS} ${DEP_LIBS}) 

SET(BINARY_NAME trainSVM)
add_executable(${BINARY_NAME} trainSVM.cpp)
target_link_libraries(${BINARY_NAME} ${AttentionSegmentation_DEPS} ${DEP_LIBS}) 

SET(BINARY_NAME segment)
add_executable(${BINARY_NAME} segment.cpp)
target_link_libraries(${BINARY_NAME} ${AttentionSegmentation_DEPS} ${DEP_LIBS}) 

SET(BINARY_NAME segmentAttention)
add_executable(${BINARY_NAME} segmentAttention.cpp)
target_link_libraries(${BINARY_NAME} ${AttentionSegmentation_DEPS} ${DEP_LIBS}) 

SET(BINARY_NAME segmentIncremental)
add_executable(${BINARY_NAME} segmentIncremental.cpp)
target_link_libraries(${BINARY_NAME} ${AttentionSegmentation_DEPS} ${DEP_LIBS}) 


INSTALL(TARGETS segmentAttention
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
