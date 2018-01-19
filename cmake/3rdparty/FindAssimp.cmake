find_path(ASSIMP_INCLUDE_DIR assimp/config.h
          PATHS /usr/local /opt /usr $ENV{ASSIMP_ROOT} ${ASSIMP_DIR}
          DOC "The path to Assimp header files"
          CMAKE_FIND_ROOT_PATH_BOTH)
find_library(ASSIMP_LIBRARY
             NAMES assimp
             PATHS /usr/local /opt /usr $ENV{ASSIMP_ROOT} ${ASSIMP_DIR}
             DOC "The Assimp library")

if(ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARY)
  # Create imported target
  v4r_add_imported_library(assimp
    IMPORTED_LOCATION ${ASSIMP_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${ASSIMP_INCLUDE_DIR}
  )
  # Find version number in pkgconfig file
  get_filename_component(_lib_dir ${ASSIMP_LIBRARY} DIRECTORY)
  find_file(_pkgconfig assimp.pc
            PATHS /usr/local /opt /usr $ENV{ASSIMP_ROOT} ${ASSIMP_DIR}
            HINTS ${_lib_dir}
            PATH_SUFFIXES lib/pkgconfig pkgconfig
            DOC "The Assimp pkfconfig file"
            CMAKE_FIND_ROOT_PATH_BOTH)
  if(_pkgconfig)
    file(STRINGS "${_pkgconfig}" _version REGEX "Version: [0-9.]+")
    if(_version)
      string(REGEX REPLACE "Version: " "" ASSIMP_VERSION ${_version})
    endif()
  endif()
  set(HAVE_ASSIMP TRUE)
else()
  set(HAVE_ASSIMP FALSE)
endif()

mark_as_advanced(ASSIMP_INCLUDE_DIR ASSIMP_LIBRARY)
