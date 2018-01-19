# ----------------------------------------------------------------------------
#  Root CMake file for V4R
#
#    From the off-tree build directory, invoke:
#      $ cmake <PATH_TO_V4R_ROOT>
#
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

# it _must_ go before project(V4R) in order to work
set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation Directory")

if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

if(POLICY CMP0022)
  cmake_policy(SET CMP0022 OLD)
endif()

if(POLICY CMP0026)
  # Disallow use of the LOCATION property for build targets.
  cmake_policy(SET CMP0026 NEW)
endif()

project(V4R CXX C)

include(cmake/V4RDebug.cmake)
include(cmake/V4RUtils.cmake)

v4r_clear_vars(V4RModules_TARGETS)

# ----------------------------------------------------------------------------
# Break in case of popular CMake configuration mistakes
# ----------------------------------------------------------------------------
if(NOT CMAKE_SIZEOF_VOID_P GREATER 0)
  message(FATAL_ERROR "CMake fails to deterimine the bitness of target platform.
  Please check your CMake and compiler installation. If you are crosscompiling then ensure that your CMake toolchain file correctly sets the compiler details.")
endif()

# ----------------------------------------------------------------------------
# Detect compiler and target platform architecture
# ----------------------------------------------------------------------------
include(cmake/V4RDetectCXXCompiler.cmake)

# Add these standard paths to the search paths for FIND_LIBRARY
# to find libraries from these locations first
if(EXISTS /lib64)
  list(APPEND CMAKE_LIBRARY_PATH /lib64)
else()
  list(APPEND CMAKE_LIBRARY_PATH /lib)
endif()

if(EXISTS /usr/lib64)
  list(APPEND CMAKE_LIBRARY_PATH /usr/lib64)
else()
  list(APPEND CMAKE_LIBRARY_PATH /usr/lib)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/V4RThirdParty.cmake)


#--------------------------------------------------------------------------------------------------#
#                                         V4R Dependencies                                         #
#--------------------------------------------------------------------------------------------------#
#                                                                                                  #
#                  Name             Default state (ON/OFF)     Prefer from source build (YES/NO)   #
#                                   Leave blank if mandatory   Leave blank if no choice            #
#                                                                                                  #
v4r_add_dependency(PCL                                                                             )
v4r_add_dependency(Boost                                                                           )
v4r_add_dependency(OpenCV                                                                          )
v4r_add_dependency(Glog                                                                            )
v4r_add_dependency(EDT                                                                             )
v4r_add_dependency(PCL_1_8                                                                         )
v4r_add_dependency(Qt               ON                                                             )
v4r_add_dependency(Eigen            ON                                                             )
v4r_add_dependency(GLM              ON                         NO                                  )
v4r_add_dependency(Ceres            ON                         YES                                 )
v4r_add_dependency(OpenGL           ON                                                             )
v4r_add_dependency(GLEW             ON                                                             )
v4r_add_dependency(Assimp           ON                                                             )
v4r_add_dependency(SiftGPU          ON                                                             )
v4r_add_dependency(LibSVM           ON                         NO                                  )
v4r_add_dependency(METSlib          ON                                                             )
v4r_add_dependency(OpenNURBS        ON                                                             )
v4r_add_dependency(Gtest           OFF                                                             )
v4r_add_dependency(Caffe            ON                                                             )
v4r_add_dependency(OpenMP           ON                                                             )
v4r_add_dependency(Radical          ON                        YES                                  )

# V4R build components
# ===================================================
V4R_OPTION(BUILD_SHARED_LIBS        "Build shared libraries (.so) instead of static ones (.a)"           ON)
V4R_OPTION(BUILD_v4r_apps           "Build utility applications (used for example to train classifiers)" ON)
V4R_OPTION(BUILD_DOCS               "Create build rules for V4R Documentation"                           ON)
V4R_OPTION(BUILD_EXAMPLES           "Build all examples"                                                 ON)
V4R_OPTION(BUILD_EVALUATION_TOOLS   "Build all evaluation tools"                                         OFF)
V4R_OPTION(BUILD_UTILITY_TOOLS      "Build all utility tools"                                            ON)
V4R_OPTION(BUILD_TESTS              "Build tests"                                                        ON)
V4R_OPTION(BUILD_WITH_DEBUG_INFO    "Include debug info into debug libs (not MSCV only)"                 ON)

# V4R installation options
# ===================================================
V4R_OPTION(INSTALL_CREATE_DISTRIB   "Change install rules to build the distribution package" OFF )
V4R_OPTION(INSTALL_C_EXAMPLES       "Install C examples"        ON )
V4R_OPTION(INSTALL_C_UTILITY_TOOLS        "Install C utility tools"        ON )
V4R_OPTION(INSTALL_C_EVALUATION_TOOLS       "Install C evaluation tools"        OFF )
V4R_OPTION(INSTALL_TO_MANGLED_PATHS "Enables mangled install paths, that help with side by side installs." OFF IF (UNIX AND NOT ANDROID AND NOT IOS AND BUILD_SHARED_LIBS) )

# V4R build options
# ===================================================
V4R_OPTION(ENABLE_PROFILING           "Enable profiling in the GCC compiler (Add flags: -g -pg)" OFF)
V4R_OPTION(ENABLE_COVERAGE            "Enable coverage collection with  GCov"                    OFF)
V4R_OPTION(ENABLE_OMIT_FRAME_POINTER  "Enable -fomit-frame-pointer for GCC"                      ON   IF (CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_COMPILER_IS_CLANGCXX) )
V4R_OPTION(ENABLE_FAST_MATH           "Enable -ffast-math (not recommended for GCC 4.6.x)"       OFF)
V4R_OPTION(ENABLE_SSE                 "Enable SSE instructions"                                  ON)
V4R_OPTION(ENABLE_SSE2                "Enable SSE2 instructions"                                 ON)
V4R_OPTION(ENABLE_SSE3                "Enable SSE3 instructions"                                 ON)
V4R_OPTION(ENABLE_SSSE3               "Enable SSSE3 instructions"                                ON)
V4R_OPTION(ENABLE_SSE41               "Enable SSE4.1 instructions"                               ON)
V4R_OPTION(ENABLE_SSE42               "Enable SSE4.2 instructions"                               ON)
V4R_OPTION(ENABLE_POPCNT              "Enable POPCNT instructions"                               OFF)
V4R_OPTION(ENABLE_AVX                 "Enable AVX instructions"                                  OFF)
V4R_OPTION(ENABLE_AVX2                "Enable AVX2 instructions"                                 OFF)
V4R_OPTION(ENABLE_FMA3                "Enable FMA3 instructions"                                 OFF)
V4R_OPTION(ENABLE_NOISY_WARNINGS      "Show all warnings even if they are too noisy"             OFF)
V4R_OPTION(V4R_WARNINGS_ARE_ERRORS    "Treat warnings as errors"                                 OFF)
V4R_OPTION(ENABLE_IMPL_COLLECTION     "Collect implementation data on function call"             OFF)
V4R_OPTION(GENERATE_ABI_DESCRIPTOR    "Generate XML file for abi_compliance_checker tool"        OFF IF UNIX)

if(ENABLE_IMPL_COLLECTION)
  add_definitions(-DCV_COLLECT_IMPL_DATA)
endif()


# ----------------------------------------------------------------------------
#  Get actual V4R version number from sources
# ----------------------------------------------------------------------------
include(cmake/V4RVersion.cmake)

# ----------------------------------------------------------------------------
#  Build & install layouts
# ----------------------------------------------------------------------------

# Save libs and executables in the same place
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin" CACHE PATH "Output directory for applications" )

set(V4R_DOC_INSTALL_PATH share/V4R/doc)
set(V4R_SAMPLES_BIN_INSTALL_PATH "samples")
set(V4R_BIN_INSTALL_PATH "bin")

set(LIBRARY_OUTPUT_PATH         "${V4R_BINARY_DIR}/lib")
set(3P_LIBRARY_OUTPUT_PATH      "${V4R_BINARY_DIR}/3rdparty/lib${LIB_SUFFIX}")

set(V4R_LIB_INSTALL_PATH         lib${LIB_SUFFIX})
set(V4R_3P_LIB_INSTALL_PATH      share/V4R/3rdparty/${V4R_LIB_INSTALL_PATH})
set(V4R_SAMPLES_SRC_INSTALL_PATH share/V4R/samples)
set(V4R_OTHER_INSTALL_PATH       share/V4R)
set(V4R_INCLUDE_INSTALL_PATH     "include")
set(V4R_3P_INCLUDE_INSTALL_PATH  share/V4R/3rdparty/${V4R_INCLUDE_INSTALL_PATH})
set(V4R_3P_INSTALL_PATH          share/V4R/3rdparty)

math(EXPR SIZEOF_VOID_P_BITS "8 * ${CMAKE_SIZEOF_VOID_P}")
if(LIB_SUFFIX AND NOT SIZEOF_VOID_P_BITS EQUAL LIB_SUFFIX)
  set(V4R_CONFIG_INSTALL_PATH lib${LIB_SUFFIX}/cmake/v4r)
else()
  set(V4R_CONFIG_INSTALL_PATH share/V4R)
endif()

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${V4R_LIB_INSTALL_PATH};${CMAKE_INSTALL_PREFIX}/${V4R_3P_LIB_INSTALL_PATH}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Postfix of so's:
set(V4R_DLLVERSION "")
set(V4R_DEBUG_POSTFIX "")

if(DEFINED CMAKE_DEBUG_POSTFIX)
  set(V4R_DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
endif()


if(INSTALL_TO_MANGLED_PATHS)
  set(V4R_INCLUDE_INSTALL_PATH ${V4R_INCLUDE_INSTALL_PATH}/v4r-${V4R_VERSION})
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_3P_LIB_INSTALL_PATH "${V4R_3P_LIB_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_SAMPLES_SRC_INSTALL_PATH "${V4R_SAMPLES_SRC_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_CONFIG_INSTALL_PATH "${V4R_CONFIG_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_DOC_INSTALL_PATH "${V4R_DOC_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_JAR_INSTALL_PATH "${V4R_JAR_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_OTHER_INSTALL_PATH "${V4R_OTHER_INSTALL_PATH}")
endif()

v4r_install_dependencies()

# ----------------------------------------------------------------------------
#  Path for build/platform -specific headers
# ----------------------------------------------------------------------------
set(V4R_CONFIG_FILE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/" CACHE PATH "Where to create the platform-dependant v4r_config.h")
v4r_include_directories(${V4R_CONFIG_FILE_INCLUDE_DIR})

# ----------------------------------------------------------------------------
#  Path for additional modules
# ----------------------------------------------------------------------------
set(V4R_EXTRA_MODULES_PATH "" CACHE PATH "Where to look for additional V4R modules")

# ----------------------------------------------------------------------------
#  Autodetect if we are in a GIT repository
# ----------------------------------------------------------------------------
find_host_package(Git QUIET)

if(GIT_FOUND)
  execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty --match "[0-9].[0-9].[0-9]*"
    WORKING_DIRECTORY "${V4R_SOURCE_DIR}"
    OUTPUT_VARIABLE V4R_VCSVERSION
    RESULT_VARIABLE GIT_RESULT
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT GIT_RESULT EQUAL 0)
    set(V4R_VCSVERSION "unknown")
  endif()
else()
  # We don't have git:
  set(V4R_VCSVERSION "unknown")
endif()


# ----------------------------------------------------------------------------
# V4R compiler and linker options
# ----------------------------------------------------------------------------
# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
if(CMAKE_GENERATOR MATCHES "Makefiles|Ninja" AND "${CMAKE_BUILD_TYPE}" STREQUAL "")
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

include(cmake/V4RCompilerOptions.cmake)


# ----------------------------------------------------------------------------
#       CHECK FOR SYSTEM LIBRARIES, OPTIONS, ETC..
# ----------------------------------------------------------------------------
find_package(PkgConfig QUIET)
include(CheckFunctionExists)
include(CheckIncludeFile)

set(V4R_LINKER_LIBS ${V4R_LINKER_LIBS} X11 Xrandr Xinerama Xcursor Xxf86vm)
set(V4R_LINKER_LIBS ${V4R_LINKER_LIBS} dl m pthread rt)

include(cmake/V4RModule.cmake)

# ----------------------------------------------------------------------------
#  Detect endianness of build platform
# ----------------------------------------------------------------------------

include(TestBigEndian)
test_big_endian(WORDS_BIGENDIAN)

# --- Doxygen for documentation ---
unset(DOXYGEN_FOUND CACHE)
if(BUILD_DOCS)
  find_package(Doxygen)
endif(BUILD_DOCS)

# Extra V4R targets: uninstall, package_source, perf, etc.
include(cmake/V4RExtraTargets.cmake)


# ----------------------------------------------------------------------------
# Process subdirectories
# ----------------------------------------------------------------------------

# TODO
# v4r.hpp and legacy headers
#add_subdirectory(include)

# V4R modules
add_subdirectory(modules)

# TODO
# Generate targets for documentation
#add_subdirectory(doc)

# TODO
# various data that is used by cv libraries and/or demo applications.
#add_subdirectory(data)

# extra applications
if(BUILD_v4r_apps)
  # TODO
  add_subdirectory(apps)
endif()

# examples
if(BUILD_EXAMPLES)
  add_subdirectory(samples)
endif()

# Figure out the list of dependencies
v4r_get_all_libs(deps_modules deps_extra deps_3rdparty)

# ----------------------------------------------------------------------------
# Finalization: generate configuration-based files
# ----------------------------------------------------------------------------

# Generate platform-dependent and configuration-dependent headers
include(cmake/V4RGenHeaders.cmake)

# TODO
# Generate v4r.pc for pkg-config command
#include(cmake/V4RGenPkgconfig.cmake)

# Generate V4RСonfig.cmake and V4RConfig-version.cmake for cmake projects
include(cmake/V4RGenConfig.cmake)

# TODO
# Generate ABI descriptor
#include(cmake/V4RGenABI.cmake)


if(NOT V4R_LICENSE_FILE)
  set(V4R_LICENSE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE)
endif()


include(cmake/V4RStatus.cmake)

#--------------------------------------------------------------------------------------------------#
#                                             Summary                                              #
#--------------------------------------------------------------------------------------------------#

status("")
status("General configuration for V4R ${V4R_VERSION} =====================================")
if(V4R_VCSVERSION)
  status("  Version control:" ${V4R_VCSVERSION})
endif()

# ========================== build platform ==========================
status("")
status("  Platform:")
status("    Host:"             ${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION} ${CMAKE_HOST_SYSTEM_PROCESSOR})
if(CMAKE_CROSSCOMPILING)
  status("    Target:"         ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION} ${CMAKE_SYSTEM_PROCESSOR})
endif()
status("    CMake:"            ${CMAKE_VERSION})
status("    CMake generator:"  ${CMAKE_GENERATOR})
status("    CMake build tool:" ${CMAKE_BUILD_TOOL})
status("    Configuration:"    ${CMAKE_BUILD_TYPE})

# ========================== C/C++ options ==========================
if(CMAKE_CXX_COMPILER_VERSION)
  set(V4R_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_CXX_COMPILER_VERSION})")
elseif(CMAKE_COMPILER_IS_CLANGCXX)
  set(V4R_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_CLANG_REGEX_VERSION})")
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(V4R_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_GCC_REGEX_VERSION})")
else()
  set(V4R_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1}")
endif()
string(STRIP "${V4R_COMPILER_STR}" V4R_COMPILER_STR)

status("")
status("  C/C++:")
status("    Built as dynamic libs:"  BUILD_SHARED_LIBS THEN YES ELSE NO)
status("    C++ Compiler:"           ${V4R_COMPILER_STR})
status("    C++ flags (Release):"    ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE})
status("    C++ flags (Debug):"      ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG})
status("    C Compiler:"             ${CMAKE_C_COMPILER} ${CMAKE_C_COMPILER_ARG1})
status("    C flags (Release):"      ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE})
status("    C flags (Debug):"        ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG})
status("    Linker flags (Release):" ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_RELEASE})
status("    Linker flags (Debug):"   ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG})

# ========================== Dependencies ============================
status("    Extra dependencies:" ${deps_extra})

# ========================== V4R modules ==========================
status("")
status("  V4R modules:")
string(REPLACE "v4r_" "" V4R_MODULES_BUILD_ST          "${V4R_MODULES_BUILD}")
string(REPLACE "v4r_" "" V4R_MODULES_DISABLED_USER_ST  "${V4R_MODULES_DISABLED_USER}")
string(REPLACE "v4r_" "" V4R_MODULES_DISABLED_FORCE_ST "${V4R_MODULES_DISABLED_FORCE}")
set(V4R_MODULES_DISABLED_AUTO_ST "")
foreach(m ${V4R_MODULES_DISABLED_AUTO})
  set(__mdeps "")
  foreach(d ${V4R_MODULE_${m}_DEPS})
    if(d MATCHES "^v4r_" AND NOT HAVE_${d})
      list(APPEND __mdeps ${d})
    endif()
  endforeach()
  if(__mdeps)
    list(APPEND V4R_MODULES_DISABLED_AUTO_ST "${m}(deps: ${__mdeps})")
  else()
    list(APPEND V4R_MODULES_DISABLED_AUTO_ST "${m}")
  endif()
endforeach()
string(REPLACE "v4r_" "" V4R_MODULES_DISABLED_AUTO_ST  "${V4R_MODULES_DISABLED_AUTO_ST}")

status("    To be built:"            V4R_MODULES_BUILD          THEN ${V4R_MODULES_BUILD_ST}          ELSE "-")
status("    Disabled:"               V4R_MODULES_DISABLED_USER  THEN ${V4R_MODULES_DISABLED_USER_ST}  ELSE "-")
status("    Disabled by dependency:" V4R_MODULES_DISABLED_AUTO  THEN ${V4R_MODULES_DISABLED_AUTO_ST}  ELSE "-")
status("    Unavailable:"            V4R_MODULES_DISABLED_FORCE THEN ${V4R_MODULES_DISABLED_FORCE_ST} ELSE "-")

# ========================== Third-party libraries ==========================
v4r_status("")
v4r_status("  Third-party libraries:")
v4r_status("")
v4r_status("  # Mandatory:")
v4r_status("")
foreach(_NAME ${V4R_3P_MANDATORY})
  v4r_dependency_status(${_NAME})
endforeach()
v4r_status("")
v4r_status("  # Optional:")
v4r_status("")
foreach(_NAME ${V4R_3P_OPTIONAL})
  v4r_dependency_status(${_NAME})
endforeach()
v4r_status("")

# ========================== documentation ==========================
if(BUILD_DOCS)
  status("")
  status("  Documentation:")
  status("    Doxygen:"             DOXYGEN_FOUND             THEN "${DOXYGEN_EXECUTABLE} (ver ${DOXYGEN_VERSION})" ELSE NO)
endif()

# ========================== samples and tests ==========================
status("")
status("  Tests and samples:")
status("    Tests:"             BUILD_TESTS AND HAVE_v4r_ts   THEN YES ELSE NO)
status("    C/C++ Examples:"    BUILD_EXAMPLES                THEN YES ELSE NO)

# ========================== auxiliary ==========================
status("")
status("  Install path:" "${CMAKE_INSTALL_PREFIX}")
status("")
status("  v4r_config.h is in:" "${V4R_CONFIG_FILE_INCLUDE_DIR}")
status("-----------------------------------------------------------------")
status("")

foreach(_name ${V4R_3P})
  message(STATUS "-- ${_name} : ${V4R_3P_${_name}_NAME} : ${${_name}_BUILT} : ${HAVE_${_name}}")
endforeach()

v4r_finalize_status()

# ----------------------------------------------------------------------------
# Warn in the case of in-source build
# ----------------------------------------------------------------------------
if("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
  message(WARNING "The source directory is the same as binary directory. \"make clean\" may damage the source tree")
endif()

# ----------------------------------------------------------------------------
# CPack stuff
# ----------------------------------------------------------------------------

# TODO
#include(cmake/V4RPackaging.cmake)

#--------------------------------------------------------------------------------------------------#
#                                              Debug                                               #
#--------------------------------------------------------------------------------------------------#
#
# * Debug message printing in V4R scripts:
#
#     -DPRINT_DEBUG=ON
#
# * Print all cache variables matching a certain string:
#
#     -DPRINT_CACHE=<STRING_TO_MATCH>
#
# * Print all properties of a target:
#
#     -DPRINT_TARGET=<TARGET_NAME>
#
v4r_print_cache_variables("${PRINT_CACHE}")
v4r_print_target_properties("${PRINT_TARGET}")
v4r_clear_vars(PRINT_CACHE PRINT_TARGET PRINT_DEBUG)
