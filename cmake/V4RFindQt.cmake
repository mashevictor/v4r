if(WITH_QT)
  find_package(Qt5 REQUIRED Core OpenGL Widgets)
  if(Qt5Core_FOUND)
      set(HAVE_QT TRUE)
      set(HAVE_QT_OPENGL TRUE)
  endif()
endif()

