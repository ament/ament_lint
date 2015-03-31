#
# Add a test to check the code for compliance with cpplint.
#
# :param TESTNAME: the name of the test, default: "cpplint"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_cpplint)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "cpplint")
  endif()

  if(NOT ament_cpplint_BIN)
    message(FATAL_ERROR "ament_cpplint() variable 'ament_cpplint_BIN' must not be empty")
  endif()

  # cpplint only works with Python 2
  set(cmd "python2" "${ament_cpplint_BIN}" "--xunit-file" "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xml")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_cpplint")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cpplint/${ARG_TESTNAME}.txt"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
