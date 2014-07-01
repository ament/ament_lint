#
# Add a test to check the code for compliance with clang_format.
#
# :param testname: the name of the test
# :type testname: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_clang_format testname)
  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "ament_clang_format() variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()
  if(NOT ament_clang_format_BIN)
    message(FATAL_ERROR "ament_clang_format() variable 'ament_clang_format_BIN' must not be empty")
  endif()

  set(subcmd "${ament_clang_format_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml\"")
  foreach(arg ${ARGN})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_clang_format")
  set(cmd "${PYTHON_EXECUTABLE} ${subcmd} > \"${CMAKE_BINARY_DIR}/ament_clang_format/${testname}.txt\"")
  ament_add_test(
    "${testname}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
