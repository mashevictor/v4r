SET(RTMT2_DEPS v4r_core v4r_io v4r_common v4r_tracking v4r_reconstruction v4r_keypoints v4r_features v4r_camera_tracking_and_mapping v4r_recognition v4r_registration edt ceres)
v4r_check_dependencies(${RTMT2_DEPS})

if(NOT V4R_DEPENDENCIES_FOUND)
  message(***RTMT2 does not meed dependencies*****)
  return()
endif()

v4r_include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
v4r_include_modules(${RTMT2_DEPS})

if(WIN32)
  link_directories(${CMAKE_CURRENT_BINARY_DIR})
endif()

if(HAVE_PCL)
  v4r_include_directories(${PCL_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${PCL_LIBRARIES})
endif()

if(HAVE_SIFTGPU)
  v4r_include_directories(${SIFTGPU_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${SIFTGPU_LIBRARIES})
endif()

if(HAVE_OPENCV)
  v4r_include_directories(${OPENCV_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${OPENCV_LIBRARIES})
endif()

if(HAVE_EDT)
  v4r_include_directories(${EDT_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${EDT_LIBRARIES})
endif()

if(HAVE_GLM)
  v4r_include_directories(${GLM_INCLUDE_DIRS})
else()
  message(***RTMT2 requires GLM*****)
  return()
endif()

if(HAVE_OPENGL)
  v4r_include_directories(${OPENGL_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${OPENGL_LIBRARIES})
endif()

if(HAVE_CERES)
  v4r_include_directories(${CERES_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${CERES_LIBRARIES})
endif()

if(HAVE_BOOST)
  v4r_include_directories(${BOOST_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${BOOST_LIBRARIES})
endif()

if(HAVE_OPENNI2)
  v4r_include_directories(${OPENNI2_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${OPENNI2_LIBRARIES})
endif()

set(SOURCES
        Camera.cpp
        CreateTrackingModel.cpp
        glviewer.cpp
        ObjectSegmentation.cpp
        main.cpp
        mainwindow.cpp
        params.cpp
        sensor.cpp
)

set(HEADERS
        Camera.h
        CreateTrackingModel.h
        glviewer.h
        mainwindow.h
        ObjectSegmentation.h
        OctreeVoxelCentroidContainerXYZRGB.hpp
        params.h
        sensor.h
)

set(FORMS
        mainwindow.ui
        params.ui
)

if(HAVE_QT4)
  v4r_include_directories(${QT4_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${QT4_LIBRARIES})
  add_definitions(${QT_DEFINITIONS})
  QT4_WRAP_CPP(HEADERS_MOC ${HEADERS})
  QT4_WRAP_UI(FORMS_MOC ${FORMS})
  set(QT_USE_QTOPENGL TRUE)
  include(${QT_USE_FILE})
  
  message(STATUS "QT_INCLUDE_DIR: ${QT_INCLUDE_DIR}")
  message(STATUS "QT_USE_FILE: ${QT_USE_FILE}")
  message(STATUS "QT_DEFINITIONS: ${QT_DEFINITIONS}")
  message(STATUS "QT_LIBRARIES: ${QT_LIBRARIES}")
elseif(HAVE_QT5)
  v4r_include_directories(${Qt5Widgets_INCLUDE_DIRS})
  v4r_include_directories(${Qt5OpenGL_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${Qt5Widgets_LIBRARIES})
  list(APPEND DEP_LIBS ${Qt5OpenGL_LIBRARIES})
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTOUIC ON)
endif()

find_package(PkgConfig REQUIRED)
pkg_search_module(GL REQUIRED gl)

add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
add_definitions(-Dlinux -D__x86_64__)


include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OPENNI2_INCLUDE_DIR})
add_executable(RTMT2 ${SOURCES} ${HEADERS_MOC} ${FORMS_MOC})
target_link_libraries(RTMT2 ${RTMT2_DEPS} ${DEP_LIBS} ${QT_LIBRARIES})

INSTALL(TARGETS RTMT2
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

