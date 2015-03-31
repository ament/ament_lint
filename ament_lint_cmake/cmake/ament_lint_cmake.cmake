#
# Add a test to lint the CMake code.
#
# :param TESTNAME: the name of the test, default: "lint_cmake"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_lint_cmake)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "lint_cmake")
  endif()

  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "ament_lint_cmake() variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()
  if(NOT ament_lint_cmake_BIN)
    message(FATAL_ERROR "ament_lint_cmake() variable 'ament_lint_cmake_BIN' must not be empty")
  endif()

  set(cmd "${PYTHON_EXECUTABLE}" "${ament_lint_cmake_BIN}" "--xunit-file" "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xml")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_lint_cmake")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_lint_cmake/${ARG_TESTNAME}.txt"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
