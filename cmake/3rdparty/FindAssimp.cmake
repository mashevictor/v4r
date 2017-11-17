v4r_clear_vars(ASSIMP_INCLUDE_PATH ASSIMP_LIBRARIES ASSIMP_VERSION ASSIMP_INCLUDE_DIRS)

find_path(ASSIMP_INCLUDE_DIRS assimp/config.h
          PATHS /usr/local /opt /usr $ENV{ASSIMP_ROOT} ${ASSIMP_DIR}
          DOC "The path to Assimp header files"
          CMAKE_FIND_ROOT_PATH_BOTH)
find_library(ASSIMP_LIBRARIES
             NAMES assimp
             PATHS /usr/local /opt /usr $ENV{ASSIMP_ROOT} ${ASSIMP_DIR}
             DOC "The Assimp library")
if(ASSIMP_INCLUDE_DIRS AND ASSIMP_LIBRARIES)
  # Find version number in pkgconfig file
  get_filename_component(_lib_dir ${ASSIMP_LIBRARIES} DIRECTORY)
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
