# Search packages for host system instead of packages for target system
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

# assert macro
# Note: it doesn't support lists in arguments
# Usage samples:
#   v4r_assert(MyLib_FOUND)
#   v4r_assert(DEFINED MyLib_INCLUDE_DIRS)
macro(v4r_assert)
  if(NOT (${ARGN}))
    string(REPLACE ";" " " __assert_msg "${ARGN}")
    message(AUTHOR_WARNING "Assertion failed: ${__assert_msg}")
  endif()
endmacro()

# adds include directories in such way that directories from the V4R source tree go first
function(v4r_include_directories)
  # v4r_print_debug_message("v4r_include_directories( ${ARGN} )")
  set(__add_before "")
  foreach(dir ${ARGN})
    get_filename_component(__abs_dir "${dir}" ABSOLUTE)
    if("${__abs_dir}" MATCHES "^${V4R_SOURCE_DIR}" OR "${__abs_dir}" MATCHES "^${V4R_BINARY_DIR}")
      list(APPEND __add_before "${dir}")
    elseif(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0" AND dir MATCHES "/usr/include$")
      # workaround for GCC 6.x bug
    else()
      include_directories(AFTER SYSTEM "${dir}")
    endif()
  endforeach()
  include_directories(BEFORE ${__add_before})
endfunction()

# adds include directories in such way that directories from the V4R source tree go first
function(v4r_target_include_directories target)
  set(__params_private "")
  set(__params_system "")
  foreach(dir ${ARGN})
    get_filename_component(__abs_dir "${dir}" ABSOLUTE)
    if("${__abs_dir}" MATCHES "^${V4R_SOURCE_DIR}" OR "${__abs_dir}" MATCHES "^${V4R_BINARY_DIR}")
      list(APPEND __params_private "${__abs_dir}")
    else()
      list(APPEND __params_system "${dir}")
    endif()
  endforeach()
  if(TARGET ${target})
    if(__params_private)
      target_include_directories(${target} PRIVATE ${__params_private})
    endif()
    if(__params_system)
      target_include_directories(${target} SYSTEM PRIVATE ${__params_system})
    endif()
  else()
    v4r_append(V4R_TARGET_SYSTEM_INCLUDE_DIRS_${target} ${__params_system})
    v4r_append(V4R_TARGET_PRIVATE_INCLUDE_DIRS_${target} ${__params_private})
  endif()
endfunction()

# clears all passed variables
macro(v4r_clear_vars)
  foreach(_var ${ARGN})
    unset(${_var})
    unset(${_var} CACHE)
  endforeach()
endmacro()

set(V4R_COMPILER_FAIL_REGEX
    "command line option .* is valid for .* but not for C\\+\\+" # GNU
    "command line option .* is valid for .* but not for C" # GNU
    "unrecognized .*option"                     # GNU
    "unknown .*option"                          # Clang
    "ignoring unknown option"                   # MSVC
    "warning D9002"                             # MSVC, any lang
    "option .*not supported"                    # Intel
    "[Uu]nknown option"                         # HP
    "[Ww]arning: [Oo]ption"                     # SunPro
    "command option .* is not recognized"       # XL
    "not supported in this configuration, ignored"       # AIX (';' is replaced with ',')
    "File with unknown suffix passed to linker" # PGI
    "WARNING: unknown flag:"                    # Open64
  )

MACRO(v4r_check_compiler_flag LANG FLAG RESULT)
  set(_fname "${ARGN}")
  if(NOT DEFINED ${RESULT})
    if(_fname)
      # nothing
    elseif("_${LANG}_" MATCHES "_CXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.cxx")
      if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        FILE(WRITE "${_fname}" "int main() { return 0; }\n")
      else()
        FILE(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      endif()
    elseif("_${LANG}_" MATCHES "_C_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.c")
      if("${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        FILE(WRITE "${_fname}" "int main(void) { return 0; }\n")
      else()
        FILE(WRITE "${_fname}" "#pragma\nint main(void) { return 0; }\n")
      endif()
    elseif("_${LANG}_" MATCHES "_OBJCXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.mm")
      if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        FILE(WRITE "${_fname}" "int main() { return 0; }\n")
      else()
        FILE(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      endif()
    else()
      unset(_fname)
    endif()
    if(_fname)
      if(NOT "x${ARGN}" STREQUAL "x")
        file(RELATIVE_PATH __msg "${CMAKE_SOURCE_DIR}" "${ARGN}")
        set(__msg " (check file: ${__msg})")
      else()
        set(__msg "")
      endif()
      MESSAGE(STATUS "Performing Test ${RESULT}${__msg}")
      TRY_COMPILE(${RESULT}
        "${CMAKE_BINARY_DIR}"
        "${_fname}"
        CMAKE_FLAGS "-DCMAKE_EXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"   # CMP0056 do this on new CMake
        COMPILE_DEFINITIONS "${FLAG}"
        OUTPUT_VARIABLE OUTPUT)

      if(${RESULT})
        string(REPLACE ";" "," OUTPUT_LINES "${OUTPUT}")
        string(REPLACE "\n" ";" OUTPUT_LINES "${OUTPUT_LINES}")
        foreach(_regex ${V4R_COMPILER_FAIL_REGEX})
          if(NOT ${RESULT})
            break()
          endif()
          foreach(_line ${OUTPUT_LINES})
            if("${_line}" MATCHES "${_regex}")
              file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
                  "Build output check failed:\n"
                  "    Regex: '${_regex}'\n"
                  "    Output line: '${_line}'\n")
              set(${RESULT} 0)
              break()
            endif()
          endforeach()
        endforeach()
      endif()

      IF(${RESULT})
        SET(${RESULT} 1 CACHE INTERNAL "Test ${RESULT}")
        MESSAGE(STATUS "Performing Test ${RESULT} - Success")
      ELSE(${RESULT})
        MESSAGE(STATUS "Performing Test ${RESULT} - Failed")
        SET(${RESULT} "" CACHE INTERNAL "Test ${RESULT}")
        file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
            "Compilation failed:\n"
            "    source file: '${_fname}'\n"
            "    check option: '${FLAG}'\n"
            "===== BUILD LOG =====\n"
            "${OUTPUT}\n"
            "===== END =====\n\n")
      ENDIF(${RESULT})
    else()
      SET(${RESULT} 0)
    endif()
  endif()
ENDMACRO()

macro(v4r_check_flag_support lang flag varname)
  if(CMAKE_BUILD_TYPE)
    set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
  endif()

  if("_${lang}_" MATCHES "_CXX_")
    set(_lang CXX)
  elseif("_${lang}_" MATCHES "_C_")
    set(_lang C)
  elseif("_${lang}_" MATCHES "_OBJCXX_")
    set(_lang OBJCXX)
  else()
    set(_lang ${lang})
  endif()

  string(TOUPPER "${flag}" ${varname})
  string(REGEX REPLACE "^(/|-)" "HAVE_${_lang}_" ${varname} "${${varname}}")
  string(REGEX REPLACE " -|-|=| |\\." "_" ${varname} "${${varname}}")

  v4r_check_compiler_flag("${_lang}" "${ARGN} ${flag}" ${${varname}})
endmacro()

# turns off warnings
macro(v4r_warnings_disable)
  if(NOT ENABLE_NOISY_WARNINGS)
    set(_flag_vars "")
    set(_msvc_warnings "")
    set(_gxx_warnings "")
    foreach(arg ${ARGN})
      if(arg MATCHES "^CMAKE_")
        list(APPEND _flag_vars ${arg})
      elseif(arg MATCHES "^/wd")
        list(APPEND _msvc_warnings ${arg})
      elseif(arg MATCHES "^-W")
        list(APPEND _gxx_warnings ${arg})
      endif()
    endforeach()
    if(MSVC AND _msvc_warnings AND _flag_vars)
      foreach(var ${_flag_vars})
        foreach(warning ${_msvc_warnings})
          set(${var} "${${var}} ${warning}")
        endforeach()
      endforeach()
    elseif((CMAKE_COMPILER_IS_GNUCXX OR (UNIX AND CV_ICC)) AND _gxx_warnings AND _flag_vars)
      foreach(var ${_flag_vars})
        foreach(warning ${_gxx_warnings})
          if(NOT warning MATCHES "^-Wno-")
            string(REPLACE "${warning}" "" ${var} "${${var}}")
            string(REPLACE "-W" "-Wno-" warning "${warning}")
          endif()
          v4r_check_flag_support(${var} "${warning}" _varname)
          if(${_varname})
            set(${var} "${${var}} ${warning}")
          endif()
        endforeach()
      endforeach()
    endif()
    unset(_flag_vars)
    unset(_msvc_warnings)
    unset(_gxx_warnings)
  endif(NOT ENABLE_NOISY_WARNINGS)
endmacro()

# Provides an option that the user can optionally select.
# Can accept condition to control when option is available for user.
# Usage:
#   option(<option_variable> "help string describing the option" <initial value or boolean expression> [IF <condition>])
macro(V4R_OPTION variable description value)
  set(__value ${value})
  set(__condition "")
  set(__varname "__value")
  foreach(arg ${ARGN})
    if(arg STREQUAL "IF" OR arg STREQUAL "if")
      set(__varname "__condition")
    else()
      list(APPEND ${__varname} ${arg})
    endif()
  endforeach()
  unset(__varname)
  if(__condition STREQUAL "")
    set(__condition 2 GREATER 1)
  endif()

  if(${__condition})
    if(__value MATCHES ";")
      if(${__value})
        option(${variable} "${description}" ON)
      else()
        option(${variable} "${description}" OFF)
      endif()
    elseif(DEFINED ${__value})
      if(${__value})
        option(${variable} "${description}" ON)
      else()
        option(${variable} "${description}" OFF)
      endif()
    else()
      option(${variable} "${description}" ${__value})
    endif()
  else()
    unset(${variable} CACHE)
  endif()
  unset(__condition)
  unset(__value)
endmacro()


set(V4R_BUILD_INFO_FILE "${V4R_BINARY_DIR}/version_string.tmp")
file(REMOVE "${V4R_BUILD_INFO_FILE}")
function(v4r_output_status msg)
  message(STATUS "${msg}")
  string(REPLACE "\\" "\\\\" msg "${msg}")
  string(REPLACE "\"" "\\\"" msg "${msg}")
  file(APPEND "${V4R_BUILD_INFO_FILE}" "\"${msg}\\n\"\n")
endfunction()

macro(v4r_finalize_status)
  if(NOT V4R_SKIP_STATUS_FINALIZATION)
    if(DEFINED V4R_MODULE_v4r_core_BINARY_DIR)
      execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different "${V4R_BUILD_INFO_FILE}" "${V4R_MODULE_v4r_core_BINARY_DIR}/version_string.inc" OUTPUT_QUIET)
    endif()
  endif()
endmacro()


# Status report function.
# Automatically align right column and selects text based on condition.
# Usage:
#   status(<text>)
#   status(<heading> <value1> [<value2> ...])
#   status(<heading> <condition> THEN <text for TRUE> ELSE <text for FALSE> )
function(status text)
  set(status_cond)
  set(status_then)
  set(status_else)

  set(status_current_name "cond")
  foreach(arg ${ARGN})
    if(arg STREQUAL "THEN")
      set(status_current_name "then")
    elseif(arg STREQUAL "ELSE")
      set(status_current_name "else")
    else()
      list(APPEND status_${status_current_name} ${arg})
    endif()
  endforeach()

  if(DEFINED status_cond)
    set(status_placeholder_length 32)
    string(RANDOM LENGTH ${status_placeholder_length} ALPHABET " " status_placeholder)
    string(LENGTH "${text}" status_text_length)
    if(status_text_length LESS status_placeholder_length)
      string(SUBSTRING "${text}${status_placeholder}" 0 ${status_placeholder_length} status_text)
    elseif(DEFINED status_then OR DEFINED status_else)
      v4r_output_status("${text}")
      set(status_text "${status_placeholder}")
    else()
      set(status_text "${text}")
    endif()

    if(DEFINED status_then OR DEFINED status_else)
      if(${status_cond})
        string(REPLACE ";" " " status_then "${status_then}")
        string(REGEX REPLACE "^[ \t]+" "" status_then "${status_then}")
        v4r_output_status("${status_text} ${status_then}")
      else()
        string(REPLACE ";" " " status_else "${status_else}")
        string(REGEX REPLACE "^[ \t]+" "" status_else "${status_else}")
        v4r_output_status("${status_text} ${status_else}")
      endif()
    else()
      string(REPLACE ";" " " status_cond "${status_cond}")
      string(REGEX REPLACE "^[ \t]+" "" status_cond "${status_cond}")
      v4r_output_status("${status_text} ${status_cond}")
    endif()
  else()
    v4r_output_status("${text}")
  endif()
endfunction()


# remove all matching elements from the list
macro(v4r_list_filterout lst regex)
  foreach(item ${${lst}})
    if(item MATCHES "${regex}")
      list(REMOVE_ITEM ${lst} "${item}")
    endif()
  endforeach()
endmacro()


# stable & safe duplicates removal macro
macro(v4r_list_unique __lst)
  if(${__lst})
    list(REMOVE_DUPLICATES ${__lst})
  endif()
endmacro()


# safe list reversal macro
macro(v4r_list_reverse __lst)
  if(${__lst})
    list(REVERSE ${__lst})
  endif()
endmacro()


# safe list sorting macro
macro(v4r_list_sort __lst)
  if(${__lst})
    list(SORT ${__lst})
  endif()
endmacro()


# add prefix to each item in the list
macro(v4r_list_add_prefix LST PREFIX)
  set(__tmp "")
  foreach(item ${${LST}})
    list(APPEND __tmp "${PREFIX}${item}")
  endforeach()
  set(${LST} ${__tmp})
  unset(__tmp)
endmacro()


# add suffix to each item in the list
macro(v4r_list_add_suffix LST SUFFIX)
  set(__tmp "")
  foreach(item ${${LST}})
    list(APPEND __tmp "${item}${SUFFIX}")
  endforeach()
  set(${LST} ${__tmp})
  unset(__tmp)
endmacro()


# gets and removes the first element from list
macro(v4r_list_pop_front LST VAR)
  if(${LST})
    list(GET ${LST} 0 ${VAR})
    list(REMOVE_AT ${LST} 0)
  else()
    set(${VAR} "")
  endif()
endmacro()


# simple regex escaping routine (does not cover all cases!!!)
macro(v4r_regex_escape var regex)
  string(REGEX REPLACE "([+.*^$])" "\\\\1" ${var} "${regex}")
endmacro()


# convert list of paths to full paths
macro(v4r_convert_to_full_paths VAR)
  if(${VAR})
    set(__tmp "")
    foreach(path ${${VAR}})
      get_filename_component(${VAR} "${path}" ABSOLUTE)
      list(APPEND __tmp "${${VAR}}")
    endforeach()
    set(${VAR} ${__tmp})
    unset(__tmp)
  endif()
endmacro()


# convert list of paths to libraries names without lib prefix
macro(v4r_convert_to_lib_name var)
  set(tmp "")
  foreach(path ${ARGN})
    get_filename_component(tmp_name "${path}" NAME)
    v4r_get_libname(tmp_name "${tmp_name}")
    list(APPEND tmp "${tmp_name}")
  endforeach()
  set(${var} ${tmp} PARENT_SCOPE)
endmacro()


# add install command
function(v4r_install_target)
  install(TARGETS ${ARGN})

  set(isPackage 0)
  unset(__package)
  unset(__target)
  foreach(e ${ARGN})
    if(NOT DEFINED __target)
      set(__target "${e}")
    endif()
    if(isPackage EQUAL 1)
      set(__package "${e}")
      break()
    endif()
    if(e STREQUAL "EXPORT")
      set(isPackage 1)
    endif()
  endforeach()

  if(DEFINED __package)
    list(APPEND ${__package}_TARGETS ${__target})
    set(${__package}_TARGETS "${${__package}_TARGETS}" CACHE INTERNAL "List of ${__package} targets")
  endif()
endfunction()


# read set of version defines from the header file
macro(v4r_parse_header FILENAME FILE_VAR)
  set(vars_regex "")
  set(__parnet_scope OFF)
  set(__add_cache OFF)
  foreach(name ${ARGN})
    if("${name}" STREQUAL "PARENT_SCOPE")
      set(__parnet_scope ON)
    elseif("${name}" STREQUAL "CACHE")
      set(__add_cache ON)
    elseif(vars_regex)
      set(vars_regex "${vars_regex}|${name}")
    else()
      set(vars_regex "${name}")
    endif()
  endforeach()
  if(EXISTS "${FILENAME}")
    file(STRINGS "${FILENAME}" ${FILE_VAR} REGEX "#define[ \t]+(${vars_regex})[ \t]+[0-9]+" )
  else()
    unset(${FILE_VAR})
  endif()
  foreach(name ${ARGN})
    if(NOT "${name}" STREQUAL "PARENT_SCOPE" AND NOT "${name}" STREQUAL "CACHE")
      if(${FILE_VAR})
        if(${FILE_VAR} MATCHES ".+[ \t]${name}[ \t]+([0-9]+).*")
          string(REGEX REPLACE ".+[ \t]${name}[ \t]+([0-9]+).*" "\\1" ${name} "${${FILE_VAR}}")
        else()
          set(${name} "")
        endif()
        if(__add_cache)
          set(${name} ${${name}} CACHE INTERNAL "${name} parsed from ${FILENAME}" FORCE)
        elseif(__parnet_scope)
          set(${name} "${${name}}" PARENT_SCOPE)
        endif()
      else()
        unset(${name} CACHE)
      endif()
    endif()
  endforeach()
endmacro()

function(v4r_target_link_libraries target)
  set(LINK_DEPS ${ARGN})
  target_link_libraries(${target} ${LINK_DEPS})
endfunction()

function(_v4r_append_target_includes target)
  if(V4R_TARGET_SYSTEM_INCLUDE_DIRS_${target})
    target_include_directories(${target} SYSTEM PRIVATE ${V4R_TARGET_SYSTEM_INCLUDE_DIRS_${target}})
    unset(V4R_TARGET_SYSTEM_INCLUDE_DIRS_${target} CACHE)
  endif()
  if(V4R_TARGET_PRIVATE_INCLUDE_DIRS_${target})
    target_include_directories(${target} PRIVATE ${V4R_TARGET_PRIVATE_INCLUDE_DIRS_${target}})
    unset(V4R_TARGET_PRIVATE_INCLUDE_DIRS_${target} CACHE)
  endif()
endfunction()

function(v4r_add_executable target)
  add_executable(${target} ${ARGN})
  _v4r_append_target_includes(${target})
endfunction()

function(v4r_add_library target)
  set(cuda_objs "")
  if(HAVE_CUDA)
    set(cuda_srcs "")

    foreach(var ${ARGN})
      if(var MATCHES ".cu")
        list(APPEND cuda_srcs ${var})
      endif()
    endforeach()

    if(cuda_srcs)
      v4r_include_directories(${CUDA_INCLUDE_DIRS})
      v4r_cuda_compile(cuda_objs ${lib_cuda_srcs} ${lib_cuda_hdrs})
    endif()
    set(V4R_MODULE_${target}_CUDA_OBJECTS ${cuda_objs} CACHE INTERNAL "Compiled CUDA object files")
  endif()

  add_library(${target} ${ARGN} ${cuda_objs})

  _v4r_append_target_includes(${target})
endfunction()

macro(v4r_get_libname var_name)
  get_filename_component(__libname "${ARGN}" NAME)
  # libv4r_core.so.3.3 -> opencv_core
  string(REGEX REPLACE "^lib(.+)\\.(a|so)(\\.[.0-9]+)?$" "\\1" __libname "${__libname}")
  set(${var_name} "${__libname}")
endmacro()

# build the list of v4r libs and dependencies for all modules
#  _modules - variable to hold list of all modules
#  _extra - variable to hold list of extra dependencies
#  _3rdparty - variable to hold list of prebuilt 3rdparty libraries
macro(v4r_get_all_libs _modules _extra _3rdparty)
  set(${_modules} "")
  set(${_extra} "")
  set(${_3rdparty} "")
  foreach(m ${V4R_MODULES_PUBLIC})
    if(TARGET ${m})
      get_target_property(deps ${m} INTERFACE_LINK_LIBRARIES)
      if(NOT deps)
        set(deps "")
      endif()
    else()
      set(deps "")
    endif()
    list(INSERT ${_modules} 0 ${deps} ${m})
    foreach (dep ${deps} ${V4R_LINKER_LIBS})
      if (NOT DEFINED V4R_MODULE_${dep}_LOCATION)
        if (TARGET ${dep})
          # get_target_property(_output ${dep} ARCHIVE_OUTPUT_DIRECTORY)
          # if ("${_output}" STREQUAL "${3P_LIBRARY_OUTPUT_PATH}")
            # list(INSERT ${_3rdparty} 0 ${dep})
          # else()
            # list(INSERT ${_extra} 0 ${dep})
          # endif()
        else()
          list(INSERT ${_extra} 0 ${dep})
        endif()
      endif()
    endforeach()
  endforeach()

  # split 3rdparty libs and modules
  list(REMOVE_ITEM ${_modules} ${${_3rdparty}} ${${_extra}} non_empty_list)

  # convert CMake lists to makefile literals
  foreach(lst ${_modules} ${_3rdparty} ${_extra})
    v4r_list_unique(${lst})
    v4r_list_reverse(${lst})
  endforeach()
endmacro()


# Append elements to an internal cached list (help string is preserved)
# A new list is created if the variable does not exist yet
macro(v4r_append _list)
  if(NOT DEFINED ${_list})
    set(${_list} ${ARGN} CACHE INTERNAL "")
  else()
    get_property(_help_string CACHE "${_list}" PROPERTY HELPSTRING)
    list(APPEND ${_list} ${ARGN})
    set(${_list} ${${_list}} CACHE INTERNAL "${_help_string}")
  endif()
endmacro()


# Add a global imported library
# Type is automatically determined from IMPORTED_LOCATION
macro(v4r_add_imported_library _name)
  # Extract (supported) target properties
  set(options)
  set(one_value_args IMPORTED_LOCATION)
  set(multi_value_args INTERFACE_LINK_LIBRARIES INTERFACE_INCLUDE_DIRECTORIES)
  cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  # Determine library type (shared/static/interface)
  if(ARG_IMPORTED_LOCATION)
    if(ARG_IMPORTED_LOCATION MATCHES "\.so$")
      set(_type "SHARED")
    elseif(ARG_IMPORTED_LOCATION MATCHES "\.a$")
      set(_type "STATIC")
    else()
      message(WARNING "Unable to detect the type of imported library ${ARG_IMPORTED_LOCATION}, assuming SHARED")
      set(_type "SHARED")
    endif()
  else()
    set(_type "INTERFACE")
  endif()

  # Create library and set properties
  add_library(${_name} ${_type} IMPORTED GLOBAL)
  foreach(_m ${one_value_args} ${multi_value_args})
    if(ARG_${_m})
      set_property(TARGET ${_name} PROPERTY ${_m} ${ARG_${_m}})
    endif()
  endforeach()
endmacro()


# Determine the location of an imported library target
# If the target is not an INTERFACE_LIBRARY, then the output is simply the IMPORTED_LOCATION
# Otherwise, if the target has interface include directories, the first one will be output
# Otherwise, if the target has interface link libraries, the location of the first one will be output
function(v4r_get_imported_library_location _target _result)
  if(TARGET ${_target})
    get_property(_type TARGET ${_target} PROPERTY TYPE)
    if(NOT _type STREQUAL "INTERFACE_LIBRARY")
      get_property(_location TARGET ${_target} PROPERTY IMPORTED_LOCATION)
      if(_location)
        set(${_result} "${_location}" PARENT_SCOPE)
        return()
      endif()
    endif()
    get_property(_includes TARGET ${_target} PROPERTY INTERFACE_INCLUDE_DIRECTORIES SET)
    if(_includes)
      get_target_property(_includes ${_target} INTERFACE_INCLUDE_DIRECTORIES)
      if(_includes MATCHES ";")
        list(GET _includes 0 _includes)
      endif()
      set(${_result} "${_includes}" PARENT_SCOPE)
    else()
      get_property(_links TARGET ${_target} PROPERTY INTERFACE_LINK_LIBRARIES SET)
      if(_links)
        get_target_property(_links ${_target} INTERFACE_LINK_LIBRARIES)
        if(_links MATCHES ";")
          list(GET _links 0 _links)
        endif()
        v4r_get_imported_library_location(${_links} _out)
        set(${_result} "${_out}" PARENT_SCOPE)
      endif()
    endif()
  else()
    set(${_result} "" PARENT_SCOPE)
  endif()
endfunction()
