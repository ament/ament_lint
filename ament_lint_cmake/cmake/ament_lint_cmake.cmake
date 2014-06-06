#
# Add a test to lint the CMake code.
#
# :param testname: the name of the test
# :type testname: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_lint_cmake testname)
  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "ament_lint_cmake() variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()
  if(NOT ament_lint_cmake_BIN)
    message(FATAL_ERROR "ament_lint_cmake() variable 'ament_lint_cmake_BIN' must not be empty")
  endif()

  set(subcmd "${ament_lint_cmake_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml\"")
  foreach(arg ${ARGN})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_lint_cmake")
  set(cmd "${PYTHON_EXECUTABLE} ${subcmd} > \"${CMAKE_BINARY_DIR}/ament_lint_cmake/${testname}.txt\"")
  ament_add_test(
    "${testname}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
