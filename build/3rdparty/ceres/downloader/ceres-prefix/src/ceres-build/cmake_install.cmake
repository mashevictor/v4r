# Install script for directory: /home/victor/software/v4r/build/3rdparty/ceres/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/victor/software/v4r/build/3rdparty/ceres/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres" TYPE FILE FILES
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/autodiff_local_parameterization.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/normal_prior.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/cost_function.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/dynamic_numeric_diff_cost_function.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/cubic_interpolation.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/covariance.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/sized_cost_function.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/loss_function.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/iteration_callback.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/numeric_diff_options.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/types.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/gradient_checker.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/problem.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/cost_function_to_functor.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/solver.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/fpclassify.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/dynamic_autodiff_cost_function.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/gradient_problem.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/ordered_groups.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/gradient_problem_solver.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/dynamic_cost_function.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/autodiff_cost_function.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/c_api.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/conditioned_cost_function.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/dynamic_cost_function_to_functor.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/version.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/crs_matrix.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/jet.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/rotation.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/ceres.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/local_parameterization.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/numeric_diff_cost_function.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/port.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/fixed_array.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/manual_constructor.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/disable_warnings.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/numeric_diff.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/macros.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/autodiff.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/scoped_ptr.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/variadic_evaluate.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/eigen.h"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/include/ceres/internal/reenable_warnings.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES "/home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build/config/ceres/internal/config.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake"
         "/home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets-release.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE RENAME "CeresConfig.cmake" FILES "/home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build/CeresConfig-install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES
    "/home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build/CeresConfigVersion.cmake"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/cmake/FindEigen.cmake"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/cmake/FindGlog.cmake"
    "/home/victor/software/v4r/build/3rdparty/ceres/src/cmake/FindGflags.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build/internal/ceres/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
