if(BUILD_RADICAL)
  set(radical_DIR ${V4R_3P_RADICAL_INSTALL_DIR}/lib/cmake/radical)
endif()

find_package(radical CONFIG)

if(radical_FOUND)
  set(RADICAL_VERSION "${radical_VERSION}")
  set(HAVE_RADICAL TRUE)
else()
  set(HAVE_RADICAL FALSE)
endif()
