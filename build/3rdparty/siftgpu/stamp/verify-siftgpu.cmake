set(file "/home/victor/SiftGPU-V400")
message(STATUS "verifying file...
     file='${file}'")
set(expect_value "a1c361e34a05dbfc4acc42f9004ba8ac")
set(attempt 0)
set(succeeded 0)
while(${attempt} LESS 0 OR ${attempt} EQUAL 0 AND NOT ${succeeded})
  file(SHA1 "${file}" actual_value)
  if("${actual_value}" STREQUAL "${expect_value}")
    set(succeeded 1)
  elseif(${attempt} LESS 0)
    message(STATUS "SHA1 hash of ${file}
does not match expected value
  expected: ${expect_value}
    actual: ${actual_value}
Retrying download.
")
    file(REMOVE "${file}")
    execute_process(COMMAND ${CMAKE_COMMAND} -P "")
  endif()
  math(EXPR attempt "${attempt} + 1")
endwhile()

if(${succeeded})
  message(STATUS "verifying file... done")
else()
  message(FATAL_ERROR "error: SHA1 hash of
  ${file}
does not match expected value
  expected: ${expect_value}
    actual: ${actual_value}
")
endif()
