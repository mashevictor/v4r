set(V4R_3P_ALL "" CACHE INTERNAL "List of all V4R third-party dependencies")
set(V4R_3P_MANDATORY "" CACHE INTERNAL "List of mandatory V4R third-party dependencies")
set(V4R_3P_OPTIONAL "" CACHE INTERNAL "List of optional V4R third-party dependencies")
set(V4R_3P_BINARY_DIR "${CMAKE_BINARY_DIR}/3rdparty" CACHE INTERNAL "Binary directory for the bundled third-party dependencies")
set(V4R_3P_SOURCE_DIR "${CMAKE_SOURCE_DIR}/3rdparty" CACHE INTERNAL "Source directory for the bundled third-party dependencies")

# Add a third-party dependency to the build system
# Corresponding files should be in the cmake/3rdparty
macro(v4r_add_dependency _Name)
  string(TOUPPER ${_Name} _NAME)
  string(TOLOWER ${_Name} _name)

  # Parse options
  unset(_default_state)
  unset(_prefer_bundled)
  foreach(_arg ${ARGN})
    if("${_arg}" STREQUAL "ON")
      set(_default_state ON)
    endif()
    if("${_arg}" STREQUAL "OFF")
      set(_default_state OFF)
    endif()
    if("${_arg}" STREQUAL "YES")
      set(_prefer_bundled ON)
    endif()
    if("${_arg}" STREQUAL "NO")
      set(_prefer_bundled OFF)
    endif()
  endforeach()

  # Check find/build scripts
  set(_find_script "${CMAKE_SOURCE_DIR}/cmake/3rdparty/Find${_Name}.cmake")
  set(_build_script "${CMAKE_SOURCE_DIR}/cmake/3rdparty/Build${_Name}.cmake")
  if(NOT EXISTS ${_find_script})
    message(FATAL_ERROR "Can not add dependency ${_Name} because it has no finder script!")
  endif()
  if(NOT EXISTS ${_build_script})
    if(DEFINED _prefer_bundled)
      message(FATAL_ERROR "Can not choose preferred source for dependency ${_Name} because it has no build script!")
    endif()
    set(_buildable OFF)
  else()
    set(_buildable ON)
  endif()

  # Create an option to toggle dependency
  if(NOT DEFINED _default_state)
    # Mandatory dependency, force WITH_* to ON and hide it from the user
    set(WITH_${_NAME} ON CACHE INTERNAL "" FORCE)
    set(_mandatory ON)
  else()
    # Not mandatory, create WITH_* option if does not exist yet
    if(NOT DEFINED WITH_${_NAME}})
      option(WITH_${_NAME} "Use ${_Name} dependency" ${_default_state})
    endif()
    set(_mandatory OFF)
  endif()

  # Create an option to toggle system/bundled dependency source
  if(NOT DEFINED _prefer_bundled)
    # The source can not be chosen, force BUILD_* to a fixed value
    set(BUILD_${_NAME} ${_buildable} CACHE INTERNAL "" FORCE)
  else()
    # Allow the user to choose source with BUILD_* option (if does not exist yet)
    if(NOT DEFINED BUILD_${_NAME}})
      option(BUILD_${_NAME} "Build ${_Name} dependency from source" ${_prefer_bundled})
    endif()
  endif()

  # Create and/or update internal config variables pertaining to this dependency
  set(V4R_3P_${_NAME}_NAME "${_Name}" CACHE INTERNAL "Pretty name of ${_Name} dependency")
  set(V4R_3P_${_NAME}_BINARY_DIR "${V4R_3P_BINARY_DIR}/${_name}" CACHE INTERNAL "Binary directory for ${_Name} dependency")
  set(V4R_3P_${_NAME}_INSTALL_DIR "${V4R_3P_${_NAME}_BINARY_DIR}/install" CACHE INTERNAL "Install directory for ${_Name} dependency")

  # Update lists
  v4r_append(V4R_3P_ALL ${_NAME})
  if(_mandatory)
    v4r_append(V4R_3P_MANDATORY ${_NAME})
  else()
    v4r_append(V4R_3P_OPTIONAL ${_NAME})
  endif()

  # Build and find the dependency as needed
  if(WITH_${_NAME})
    if(BUILD_${_NAME} AND (NOT V4R_3P_${_NAME}_BUILT OR NOT EXISTS ${V4R_3P_${_NAME}_INSTALL_DIR}))
      include(${_build_script})
    endif()
    include(${_find_script})
  endif()

  if(_mandatory AND NOT HAVE_${_NAME})
    message(FATAL_ERROR "Mandatory dependency ${_Name} is not found!")
  endif()
endmacro()


# Create CMake rules to install third-party dependencies
macro(v4r_install_dependencies)
  foreach(_NAME ${V4R_3P_ALL})
    if(WITH_${_NAME} AND BUILD_${_NAME})
      install(DIRECTORY ${V4R_3P_${_NAME}_INSTALL_DIR}/ DESTINATION ${V4R_3P_INSTALL_PATH})
    endif()
  endforeach()
endmacro()


# Build an external project
# This is similar to the built-in CMake ExternalProject_Add
macro(v4r_build_external_project _Name)
  string(TOUPPER ${_Name} _NAME)
  string(TOLOWER ${_Name} _name)

  v4r_clear_vars(V4R_3P_${_NAME}_BUILT)

  set(_binary_dir "  BINARY_DIR \"${V4R_3P_${_NAME}_BUILD_DIR}\"\n")
  set(_bundled FALSE)
  set(_in_source FALSE)
  set(_params "")
  set(_separator "\n  ")
  foreach(_arg ${ARGN})
    # BUNDLED is our custom keyword argument
    if(_arg STREQUAL "BUNDLED")
      set(_bundled TRUE)
    else()
      if(_arg STREQUAL "NO")
        set(_arg "\"\"")
      elseif(_arg STREQUAL "BUILD_IN_SOURCE")
        set(_in_source TRUE)
      elseif(_arg STREQUAL "CMAKE_ARGS")
        set(_arg "${_arg} -G \"${CMAKE_GENERATOR}\"")
      endif()
      set(_params "${_params}${_separator}${_arg}")
      if(_separator STREQUAL " ")
        set(_separator "\n  ")
      else()
        set(_separator " ")
      endif()
    endif()
  endforeach()

  # Configure directories
  if(_bundled)
    set(V4R_3P_${_NAME}_SOURCE_DIR "${V4R_3P_SOURCE_DIR}/${_name}" CACHE INTERNAL "Source directory for ${_Name} dependency")
  else()
    set(V4R_3P_${_NAME}_SOURCE_DIR "${V4R_3P_${_NAME}_BINARY_DIR}/src" CACHE INTERNAL "Source directory for ${_Name} dependency")
  endif()
  set(V4R_3P_${_NAME}_BUILD_DIR "${V4R_3P_${_NAME}_BINARY_DIR}/build" CACHE INTERNAL "Build directory for ${_Name} dependency")
  set(_downloader_path "${V4R_3P_${_NAME}_BINARY_DIR}/downloader")

  if(_in_source)
    set(_binary_dir "  BINARY_DIR \"${V4R_3P_${_NAME}_SOURCE_DIR}\"\n")
  endif()

  file(MAKE_DIRECTORY "${_downloader_path}")
  file(WRITE "${_downloader_path}/CMakeLists.txt"
    "cmake_minimum_required(VERSION 2.8.12)\n"
    "project(${_name})\n"
    "include(ExternalProject)\n"
    "ExternalProject_Add(${_name}${_params}\n"
    "  TIMEOUT 30\n"
    "  TMP_DIR \"${V4R_3P_${_NAME}_BINARY_DIR}/tmp\"\n"
    "  STAMP_DIR \"${V4R_3P_${_NAME}_BINARY_DIR}/stamp\"\n"
    "  DOWNLOAD_DIR \"${V4R_3P_BINARY_DIR}/download\"\n"
    "  SOURCE_DIR \"${V4R_3P_${_NAME}_SOURCE_DIR}\"\n"
    ${_binary_dir}
    "  INSTALL_DIR \"${V4R_3P_${_NAME}_INSTALL_DIR}\"\n"
    ")\n"
    "ExternalProject_Add_Step(${_name} ROBUST_PATCH\n"
    "  COMMAND \${CMAKE_COMMAND} -DWORKING_DIR=<SOURCE_DIR> -DPATCHES_DIR=${V4R_3P_SOURCE_DIR}/${_name} -P ${CMAKE_SOURCE_DIR}/cmake/scripts/ApplyPatches.cmake\n"
    "  COMMENT \"Performing robust patch step\"\n"
    "  DEPENDEES update\n"
    ")\n"
  )

  message(STATUS "Preparing CMake scripts for ${_Name}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${_downloader_path}"
    RESULT_VARIABLE config_result
  )
  if(NOT config_result EQUAL 0)
    message(FATAL_ERROR "Failed to prepare CMake scripts for ${_Name}")
  endif()

  message(STATUS "Configuring and building ${_Name}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} --build . --config Release
    WORKING_DIRECTORY "${_downloader_path}"
    RESULT_VARIABLE build_result
  )
  if(NOT build_result EQUAL 0)
    message(FATAL_ERROR "Failed to configure and build ${_Name}")
  endif()
  set(V4R_3P_${_NAME}_BUILT TRUE CACHE INTERNAL "${_Name} is built")
endmacro()

