#
# Add a test to perform static code analysis with bandit.
#
# :param TESTNAME: the name of the test, default: "bandit"
# :type TESTNAME: string
# :param EXCLUDE: an optional list of exclude files or directories
# :type EXCLUDE: list
# :param FORMAT: output format
# :type FORMAT: string
# :param OUTPUT: filename of output report
# :type OUTPUT: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_bandit)
  cmake_parse_arguments(ARG "" "LANGUAGE;TESTNAME" "EXCLUDE;FORMAT;OUTPUT" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "bandit")
  endif()

  find_program(ament_bandit_BIN NAMES "ament_bandit")
  if(NOT ament_bandit_BIN)
    message(FATAL_ERROR "ament_bandit() could not find program 'ament_bandit'")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(cmd "${ament_bandit_BIN}" "--xunit-file" "${result_file}")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})
  if(ARG_EXCLUDE)
    list(APPEND cmd "--exclude" "${ARG_EXCLUDE}")
  endif()
  if(ARG_FORMAT)
    list(APPEND cmd "--format" "${ARG_FORMAT}")
  endif()
  if(ARG_OUTPUT)
    list(APPEND cmd "--output" "${ARG_OUTPUT}")
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_bandit")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    TIMEOUT 300
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_bandit/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "bandit;linter"
  )
endfunction()
