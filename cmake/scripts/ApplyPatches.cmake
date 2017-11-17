file(GLOB PATCHES ${PATCHES_DIR}/*.patch)
if(PATCHES)
  list(LENGTH PATCHES NUM_PATCHES)
  list(SORT PATCHES)
  if(NUM_PATCHES EQUAL 1)
    message(STATUS "Found ${NUM_PATCHES} patch:")
  else()
    message(STATUS "Found ${NUM_PATCHES} patches:")
  endif()
  foreach(PATCH ${PATCHES})
    get_filename_component(NAME ${PATCH} NAME)
    execute_process(COMMAND patch -p0 --silent -i ${PATCH}
      WORKING_DIRECTORY "${WORKING_DIR}"
      RESULT_VARIABLE PATCH_FAILED
      ERROR_QUIET
    )
    if(PATCH_FAILED)
      message(STATUS " [x] ${NAME}")
    else()
      message(STATUS " [+] ${NAME}")
    endif()
  endforeach()
endif()
