SET(ObjectGroundTruthAnnotator_DEPS v4r_core v4r_registration v4r_io v4r_common v4r_features v4r_keypoints v4r_reconstruction v4r_recognition edt)
v4r_check_dependencies(${ObjectGroundTruthAnnotator_DEPS})

if(NOT V4R_DEPENDENCIES_FOUND)
  message(***ObjectGroundTruthAnnotator does not meet dependencies*****)
  return()
endif()

v4r_include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
v4r_include_modules(${ObjectGroundTruthAnnotator_DEPS})

if(NOT HAVE_QT)
  message(***ObjectGroundTruthAnnotator requires Qt*****)
  return()
endif()

set(FORMS
        mainwindow.ui
        params.ui
)

qt5_wrap_ui(FORMS_MOC ${FORMS})

#add_definitions(-std=c++0x)
add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
add_definitions(-Dlinux -D__x86_64__)



add_executable(GT6DOF GT6DOF.cpp ${foo_MOC_SRCS} main_window.cpp)
if(VTK_VERSION_MAJOR LESS 6)
target_link_libraries(GT6DOF ${ObjectGroundTruthAnnotator_DEPS} QVTK qt  ${GL_LIBRARIES})
else ()
target_link_libraries(GT6DOF ${ObjectGroundTruthAnnotator_DEPS} qt ${GL_LIBRARIES})
endif()

INSTALL(TARGETS GT6DOF
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
