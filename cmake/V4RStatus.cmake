# Macros/functions for pretty printing status and configuration reports

string(ASCII 27 __esc)
set(__reset   "${__esc}[m")
set(__bold    "${__esc}[1m")
set(__red     "${__esc}[31m")
set(__green   "${__esc}[32m")
set(__orange  "${__esc}[33m")
set(__magenta "${__esc}[35m")
set(__on    "☑")
set(__off   "☒")


# Print status (availability, version, etc) of a third-party dependency.
function(v4r_dependency_status _NAME)
  string(TOLOWER ${_NAME} _name)
  set(_Name ${V4R_3P_${_NAME}_NAME})

  set(options)
  set(one_value_args INDENT)
  set(multi_value_args)
  cmake_parse_arguments(STATUS "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  set(_width 28)
  string(RANDOM LENGTH ${_width} ALPHABET " " _placeholder)
  string(LENGTH "${_Name}" _name_length)

  if(STATUS_INDENT)
    string(RANDOM LENGTH ${STATUS_INDENT} ALPHABET " " _indent)
  else()
    set(_indent "")
  endif()

  if(_name_length LESS _width)
    string(SUBSTRING "${_indent}${_Name}${_placeholder}" 0 ${_width} _text)
  else()
    set(_text "${_indent}${_Name}")
  endif()

  v4r_pad("${${_NAME}_VERSION}" 10 _version)
  if(HAVE_${_NAME})
    if(BUILD_${_NAME})
      set(_state "built")
    else()
      v4r_get_imported_library_location(${_name} _location)
      if(_location)
        set(_at " (${_location})")
      endif()
      set(_state "found${_at}")
    endif()
    set(_symbol ${__on})
  elseif(WITH_${_NAME})
    set(_state "not found")
    set(_color ${__red})
    set(_symbol ${__off})
  else()
    set(_state "disabled")
    set(_color ${__orange})
    set(_symbol ${__off})
  endif()
  message(STATUS "  ${_color}${_symbol}${__reset}  ${_text}${_version}${_color}${_state}${__reset}")
endfunction()


# Pad a string to a given width with spaces (from the right)
# If the string is longer than width, then it is NOT cropped
# Positional arguments:
#   _string : string to pad
#   _width  : desired output width
#   _result : variable where to store the output
function(v4r_pad _string _width _result)
  string(RANDOM LENGTH ${_width} ALPHABET " " _placeholder)
  string(LENGTH "${_string}" _string_length)
  if(_string_length LESS _width)
    string(SUBSTRING "${_string}${_placeholder}" 0 ${_width} _text)
    set(${_result} ${_text} PARENT_SCOPE)
  else()
    set(${_result} ${_string} PARENT_SCOPE)
  endif()
endfunction()


# Status report function.
# Automatically aligns right column and selects text based on condition.
# Usage:
#   v4r_status(<text>)
#   v4r_status(<heading> <value1> [<value2> ...])
#   v4r_status(<heading> <condition> THEN <text for TRUE> ELSE <text for FALSE>)
#
# ~ Source code copied from OpenCV
function(v4r_status text)
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
      message(STATUS "${text}")
      set(status_text "${status_placeholder}")
    else()
      set(status_text "${text}")
    endif()

    if(DEFINED status_then OR DEFINED status_else)
      if(${status_cond})
        string(REPLACE ";" " " status_then "${status_then}")
        string(REGEX REPLACE "^[ \t]+" "" status_then "${status_then}")
        message(STATUS "${status_text} ${status_then}")
      else()
        string(REPLACE ";" " " status_else "${status_else}")
        string(REGEX REPLACE "^[ \t]+" "" status_else "${status_else}")
        message(STATUS "${status_text} ${status_else}")
      endif()
    else()
      string(REPLACE ";" " " status_cond "${status_cond}")
      string(REGEX REPLACE "^[ \t]+" "" status_cond "${status_cond}")
      message(STATUS "${status_text} ${status_cond}")
    endif()
  else()
    message(STATUS "${text}")
  endif()
endfunction()
