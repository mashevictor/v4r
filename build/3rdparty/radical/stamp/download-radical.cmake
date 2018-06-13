if(EXISTS "/home/victor/software/v4r/build/3rdparty/download/1.0.0.tar.gz")
  file("SHA256" "/home/victor/software/v4r/build/3rdparty/download/1.0.0.tar.gz" hash_value)
  if("x${hash_value}" STREQUAL "x672751353e83472ba81bdde0bfad550784dd1e66502d88721aa9b406a5a90ea8")
    return()
  endif()
endif()
message(STATUS "downloading...
     src='https://github.com/taketwo/radical/archive/1.0.0.tar.gz'
     dst='/home/victor/software/v4r/build/3rdparty/download/1.0.0.tar.gz'
     timeout='30 seconds'")




file(DOWNLOAD
  "https://github.com/taketwo/radical/archive/1.0.0.tar.gz"
  "/home/victor/software/v4r/build/3rdparty/download/1.0.0.tar.gz"
  SHOW_PROGRESS
  TIMEOUT;30
  STATUS status
  LOG log)

list(GET status 0 status_code)
list(GET status 1 status_string)

if(NOT status_code EQUAL 0)
  message(FATAL_ERROR "error: downloading 'https://github.com/taketwo/radical/archive/1.0.0.tar.gz' failed
  status_code: ${status_code}
  status_string: ${status_string}
  log: ${log}
")
endif()

message(STATUS "downloading... done")
