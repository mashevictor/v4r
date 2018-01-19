find_package(Boost 1.48 COMPONENTS thread program_options serialization system filesystem regex)
if(Boost_FOUND)
  set(BOOST_VERSION "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}")
  v4r_add_imported_library(boost
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS}
    INTERFACE_LINK_LIBRARIES ${Boost_LIBRARIES}
  )
  set(HAVE_BOOST TRUE)
else()
  set(HAVE_BOOST FALSE)
endif()

