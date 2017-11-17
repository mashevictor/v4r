v4r_clear_vars(QT_INCLUDE_DIRS QT_LIBRARIES QT_VERSION)

find_package(Qt5 COMPONENTS Core OpenGL Widgets)
if(Qt5Core_FOUND)
  set(QT_LIBRARIES Qt5::Core Qt5::Widgets Qt5::OpenGL)
  set(QT_INCLUDE_DIRS "")
  set(QT_VERSION ${Qt5Core_VERSION})
  set(HAVE_QT TRUE)
else()
  set(HAVE_QT FALSE)
endif()

