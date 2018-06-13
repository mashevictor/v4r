if(EXISTS "/home/victor/software/v4r/build/3rdparty/download/ceres-solver-1.13.0.tar.gz")
  file("SHA256" "/home/victor/software/v4r/build/3rdparty/download/ceres-solver-1.13.0.tar.gz" hash_value)
  if("x${hash_value}" STREQUAL "x1DF490A197634D3AAB0A65687DECD362912869C85A61090FF66F073C967A7DCD")
    return()
  endif()
endif()
message(STATUS "downloading...
     src='http://ceres-solver.org/ceres-solver-1.13.0.tar.gz'
     dst='/home/victor/software/v4r/build/3rdparty/download/ceres-solver-1.13.0.tar.gz'
     timeout='30 seconds'")




file(DOWNLOAD
  "http://ceres-solver.org/ceres-solver-1.13.0.tar.gz"
  "/home/victor/software/v4r/build/3rdparty/download/ceres-solver-1.13.0.tar.gz"
  SHOW_PROGRESS
  TIMEOUT;30
  STATUS status
  LOG log)

list(GET status 0 status_code)
list(GET status 1 status_string)

if(NOT status_code EQUAL 0)
  message(FATAL_ERROR "error: downloading 'http://ceres-solver.org/ceres-solver-1.13.0.tar.gz' failed
  status_code: ${status_code}
  status_string: ${status_string}
  log: ${log}
")
endif()

message(STATUS "downloading... done")
