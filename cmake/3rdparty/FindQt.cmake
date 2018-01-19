find_package(Qt5 COMPONENTS Core OpenGL Widgets)
if(Qt5Core_FOUND)
  v4r_add_imported_library(qt
    INTERFACE_LINK_LIBRARIES Qt5::Core Qt5::Widgets Qt5::OpenGL
  )
  set(QT_VERSION ${Qt5Core_VERSION})
  set(HAVE_QT TRUE)
else()
  set(HAVE_QT FALSE)
endif()
