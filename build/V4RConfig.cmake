# ===================================================================================
#  The V4R CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    find_package(V4R REQUIRED)
#    include_directories(${V4R_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11
#    target_link_libraries(MY_TARGET_NAME ${V4R_LIBS})
#
#    Or you can search for specific V4R modules:
#
#    find_package(V4R REQUIRED core videoio)
#
#    If the module is found then V4R_<MODULE>_FOUND is set to TRUE.
#
#    This file will define the following variables:
#      - V4R_LIBS                     : The list of all imported targets for V4R modules.
#      - V4R_INCLUDE_DIRS             : The V4R include directories.
#      - V4R_COMPUTE_CAPABILITIES     : The version of compute capability.
#      - V4R_VERSION                  : The version of this V4R build: "2.0.6"
#      - V4R_VERSION_MAJOR            : Major version part of V4R_VERSION: "2"
#      - V4R_VERSION_MINOR            : Minor version part of V4R_VERSION: "0"
#      - V4R_VERSION_PATCH            : Patch version part of V4R_VERSION: "6"
#      - V4R_VERSION_STATUS           : Development status of this build: ""
#
#    Advanced variables:
#      - V4R_SHARED                   : Use V4R as shared library
#      - V4R_CONFIG_PATH              : Path to this V4RConfig.cmake
#      - V4R_INSTALL_PATH             : V4R location (not set on Windows)
#      - V4R_LIB_COMPONENTS           : Present V4R modules list
#      - V4R_USE_MANGLED_PATHS        : Mangled V4R path flag
#
# ===================================================================================

# Search packages for host system instead of packages for target system.
# in case of cross compilation thess macro should be defined by toolchain file

if(NOT COMMAND find_host_package)
    macro(find_host_package)
        find_package(${ARGN})
    endmacro()
endif()

if(NOT COMMAND find_host_program)
    macro(find_host_program)
        find_program(${ARGN})
    endmacro()
endif()

set(CMAKE_CXX_STANDARD 11)

# TODO All things below should be reviewed. What is about of moving this code into related modules (special vars/hooks/files)

# Version Compute Capability from which V4R has been compiled is remembered
set(V4R_COMPUTE_CAPABILITIES "")

set(V4R_CUDA_VERSION )

# Some additional settings are required if V4R is built as static libs
set(V4R_SHARED ON)

# Enables mangled install paths, that help with side by side installs
set(V4R_USE_MANGLED_PATHS FALSE)

# Extract the directory where *this* file has been installed (determined at cmake run-time)
get_filename_component(V4R_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH CACHE)
set(V4R_INSTALL_PATH "${V4R_CONFIG_PATH}/../..")
# Get the absolute path with no ../.. relative marks, to eliminate implicit linker warnings
if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_LESS 2.8)
  get_filename_component(V4R_INSTALL_PATH "${V4R_INSTALL_PATH}" ABSOLUTE)
else()
  get_filename_component(V4R_INSTALL_PATH "${V4R_INSTALL_PATH}" REALPATH)
endif()

if(NOT TARGET pcl)
  include(${CMAKE_CURRENT_LIST_DIR}/V4RDependencies.cmake)
endif()

if(NOT TARGET v4r_common)
  include(${CMAKE_CURRENT_LIST_DIR}/V4RModules.cmake)
endif()

# ======================================================
# Include directories to add to the user project:
# ======================================================

# Provide the include directories to the caller
set(V4R_INCLUDE_DIRS "/home/victor/software/v4r/build" "/home/victor/software/v4r/modules/core/include" "/home/victor/software/v4r/modules/io/include" "/home/victor/software/v4r/modules/semantic_segmentation/include" "/home/victor/software/v4r/modules/common/include" "/home/victor/software/v4r/modules/keypoints/include" "/home/victor/software/v4r/modules/ml/include" "/home/victor/software/v4r/modules/rendering/include" "/home/victor/software/v4r/modules/segmentation/include" "/home/victor/software/v4r/modules/attention_segmentation/include" "/home/victor/software/v4r/modules/change_detection/include" "/home/victor/software/v4r/modules/features/include" "/home/victor/software/v4r/modules/reconstruction/include" "/home/victor/software/v4r/modules/registration/include" "/home/victor/software/v4r/modules/tracking/include" "/home/victor/software/v4r/modules/object_modelling/include" "/home/victor/software/v4r/modules/recognition/include" "/home/victor/software/v4r/modules/apps/include" "/home/victor/software/v4r/modules/camera_tracking_and_mapping/include" "/home/victor/software/v4r/modules/surface_texturing/include")

# ======================================================
# Link directories to add to the user project:
# ======================================================

# Provide the libs directories to the caller
set(V4R_LIB_DIR_OPT  CACHE PATH "Path where release V4R libraries are located")
set(V4R_LIB_DIR_DBG  CACHE PATH "Path where debug V4R libraries are located")
set(V4R_3RDPARTY_LIB_DIR_OPT  CACHE PATH "Path where release 3rdparty V4R dependencies are located")
set(V4R_3RDPARTY_LIB_DIR_DBG  CACHE PATH "Path where debug 3rdparty V4R dependencies are located")
mark_as_advanced(FORCE V4R_LIB_DIR_OPT V4R_LIB_DIR_DBG V4R_3RDPARTY_LIB_DIR_OPT V4R_3RDPARTY_LIB_DIR_DBG V4R_CONFIG_PATH)

# ======================================================
#  Version variables:
# ======================================================
SET(V4R_VERSION        2.0.6)
SET(V4R_VERSION_MAJOR  2)
SET(V4R_VERSION_MINOR  0)
SET(V4R_VERSION_PATCH  6)
SET(V4R_VERSION_STATUS "")

# ====================================================================
# Link libraries: e.g. v4r_common;v4r_recognition; etc...
# ====================================================================

SET(V4R_LIB_COMPONENTS v4r_tracking;v4r_surface_texturing;v4r_semantic_segmentation;v4r_segmentation;v4r_rendering;v4r_registration;v4r_reconstruction;v4r_recognition;v4r_object_modelling;v4r_ml;v4r_keypoints;v4r_io;v4r_features;v4r_core;v4r_common;v4r_change_detection;v4r_camera_tracking_and_mapping;v4r_attention_segmentation;v4r_apps)

# ==============================================================
#  Extra include directories, needed by V4R new structure
# ==============================================================
#SET(V4R2_INCLUDE_DIRS )
#if(V4R2_INCLUDE_DIRS)
  #list(APPEND V4R_INCLUDE_DIRS ${V4R2_INCLUDE_DIRS})

  #set(V4R_ADD_DEBUG_RELEASE FALSE)
  #if(V4R_ADD_DEBUG_RELEASE)
    #set(V4R_LIB_DIR_OPT "${V4R_LIB_DIR_OPT}/Release")
    #set(V4R_LIB_DIR_DBG "${V4R_LIB_DIR_DBG}/Debug")
    #set(V4R_3RDPARTY_LIB_DIR_OPT "${V4R_3RDPARTY_LIB_DIR_OPT}/Release")
    #set(V4R_3RDPARTY_LIB_DIR_DBG "${V4R_3RDPARTY_LIB_DIR_DBG}/Debug")
  #endif()
#endif()

if(NOT CMAKE_VERSION VERSION_LESS "2.8.11")
  # Target property INTERFACE_INCLUDE_DIRECTORIES available since 2.8.11:
  # * http://www.cmake.org/cmake/help/v2.8.11/cmake.html#prop_tgt:INTERFACE_INCLUDE_DIRECTORIES
  foreach(__component ${V4R_LIB_COMPONENTS})
    if(TARGET ${__component})
      set_target_properties(
          ${__component}
          PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES "${V4R_INCLUDE_DIRS}"
      )
    endif()
  endforeach()
endif()

# ==============================================================
#  Form list of modules (components) to find
# ==============================================================
if(NOT V4R_FIND_COMPONENTS)
  set(V4R_FIND_COMPONENTS ${V4R_LIB_COMPONENTS})
  if(GTest_FOUND OR GTEST_FOUND)
    list(REMOVE_ITEM V4R_FIND_COMPONENTS v4r_ts)
  endif()
endif()

# expand short module names and see if requested components exist
set(V4R_FIND_COMPONENTS_ "")
foreach(__v4rcomponent ${V4R_FIND_COMPONENTS})
  if(NOT __v4rcomponent MATCHES "^v4r_")
    set(__v4rcomponent v4r_${__v4rcomponent})
  endif()
  list(FIND V4R_LIB_COMPONENTS ${__v4rcomponent} __v4rcomponentIdx)
  if(__v4rcomponentIdx LESS 0)
    #requested component is not found...
    if(V4R_FIND_REQUIRED)
      message(FATAL_ERROR "${__v4rcomponent} is required but was not found")
    elseif(NOT V4R_FIND_QUIETLY)
      message(WARNING "${__v4rcomponent} is required but was not found")
    endif()
    #indicate that module is NOT found
    string(TOUPPER "${__v4rcomponent}" __v4rcomponentUP)
    set(${__v4rcomponentUP}_FOUND "${__v4rcomponentUP}_FOUND-NOTFOUND")
  else()
    list(APPEND V4R_FIND_COMPONENTS_ ${__v4rcomponent})
    # Not using list(APPEND) here, because V4R_LIBS may not exist yet.
    # Also not clearing V4R_LIBS anywhere, so that multiple calls
    # to find_package(V4R) with different component lists add up.
    set(V4R_LIBS ${V4R_LIBS} "${__v4rcomponent}")
    #indicate that module is found
    string(TOUPPER "${__v4rcomponent}" __v4rcomponentUP)
    set(${__v4rcomponentUP}_FOUND 1)
  endif()
endforeach()
set(V4R_FIND_COMPONENTS ${V4R_FIND_COMPONENTS_})

# ==============================================================
#  Resolve dependencies
# ==============================================================
if(V4R_USE_MANGLED_PATHS)
  set(V4R_LIB_SUFFIX ".${V4R_VERSION_MAJOR}.${V4R_VERSION_MINOR}.${V4R_VERSION_PATCH}")
else()
  set(V4R_LIB_SUFFIX "")
endif()

foreach(__opttype OPT DBG)
  SET(V4R_LIBS_${__opttype} "${V4R_LIBS}")
  SET(V4R_EXTRA_LIBS_${__opttype} "")

  # CUDA
  if(V4R_CUDA_VERSION)
    if(NOT CUDA_FOUND)
      find_host_package(CUDA ${V4R_CUDA_VERSION} EXACT REQUIRED)
    else()
      if(NOT CUDA_VERSION_STRING VERSION_EQUAL V4R_CUDA_VERSION)
        message(FATAL_ERROR "V4R static library was compiled with CUDA ${V4R_CUDA_VERSION} support. Please, use the same version or rebuild V4R with CUDA ${CUDA_VERSION_STRING}")
      endif()
    endif()

    set(V4R_CUDA_LIBS_ABSPATH ${CUDA_LIBRARIES})

    if(${CUDA_VERSION} VERSION_LESS "5.5")
      list(APPEND V4R_CUDA_LIBS_ABSPATH ${CUDA_npp_LIBRARY})
    else()
      find_cuda_helper_libs(nppc)
      find_cuda_helper_libs(nppi)
      find_cuda_helper_libs(npps)
      list(APPEND V4R_CUDA_LIBS_ABSPATH ${CUDA_nppc_LIBRARY} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY})
    endif()

    if(V4R_USE_CUBLAS)
      list(APPEND V4R_CUDA_LIBS_ABSPATH ${CUDA_CUBLAS_LIBRARIES})
    endif()

    if(WIN32)
      list(APPEND V4R_CUDA_LIBS_ABSPATH ${CUDA_nvcuvenc_LIBRARIES})
    endif()

    set(V4R_CUDA_LIBS_RELPATH "")
    foreach(l ${V4R_CUDA_LIBS_ABSPATH})
      get_filename_component(_tmp ${l} PATH)
      if(NOT ${_tmp} MATCHES "-Wl.*")
          list(APPEND V4R_CUDA_LIBS_RELPATH ${_tmp})
      endif()
    endforeach()

    list(REMOVE_DUPLICATES V4R_CUDA_LIBS_RELPATH)
    link_directories(${V4R_CUDA_LIBS_RELPATH})
  endif()
endforeach()

if(CMAKE_CROSSCOMPILING AND V4R_SHARED AND (CMAKE_SYSTEM_NAME MATCHES "Linux"))
  foreach(dir ${V4R_LIB_DIR})
    set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    -Wl,-rpath-link,${dir}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link,${dir}")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,-rpath-link,${dir}")
  endforeach()
endif()

#
# Some macroses for samples
#
macro(v4r_check_dependencies)
  set(V4R_DEPENDENCIES_FOUND TRUE)
  foreach(d ${ARGN})
    if(NOT TARGET ${d})
      message(WARNING "V4R: Can't resolve dependency: ${d}")
      set(V4R_DEPENDENCIES_FOUND FALSE)
      break()
    endif()
  endforeach()
endmacro()

# adds include directories in such way that directories from the V4R source tree go first
function(v4r_include_directories)
  set(__add_before "")
  file(TO_CMAKE_PATH "${V4R_DIR}" __baseDir)
  foreach(dir ${ARGN})
    get_filename_component(__abs_dir "${dir}" ABSOLUTE)
    if("${__abs_dir}" MATCHES "^${__baseDir}")
      list(APPEND __add_before "${dir}")
    else()
      include_directories(AFTER SYSTEM "${dir}")
    endif()
  endforeach()
  include_directories(BEFORE ${__add_before})
endfunction()

macro(v4r_include_modules)
  include_directories(BEFORE "${V4R_INCLUDE_DIRS}")
endmacro()

macro(v4r_include_modules_recurse)
  include_directories(BEFORE "${V4R_INCLUDE_DIRS}")
endmacro()

macro(v4r_target_link_libraries)
  target_link_libraries(${ARGN})
endmacro()

# remove all matching elements from the list
macro(v4r_list_filterout lst regex)
  foreach(item ${${lst}})
    if(item MATCHES "${regex}")
      list(REMOVE_ITEM ${lst} "${item}")
    endif()
  endforeach()
endmacro()
