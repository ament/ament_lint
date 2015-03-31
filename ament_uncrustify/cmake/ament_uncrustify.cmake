#
# Add a test to check the code for compliance with uncrustify.
#
# :param TESTNAME: the name of the test, default: "uncrustify"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_uncrustify)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "uncrustify")
  endif()

  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "ament_uncrustify() variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()
  if(NOT ament_uncrustify_BIN)
    message(FATAL_ERROR "ament_uncrustify() variable 'ament_uncrustify_BIN' must not be empty")
  endif()

  set(cmd "${PYTHON_EXECUTABLE}" "${ament_uncrustify_BIN}" "--xunit-file" "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xml")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_uncrustify")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_uncrustify/${ARG_TESTNAME}.txt"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
