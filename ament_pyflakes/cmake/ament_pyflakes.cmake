#
# Add a test to check the code for compliance with pyflakes.
#
# :param testname: the name of the test
# :type testname: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_pyflakes testname)
  if(NOT ament_pyflakes_BIN)
    message(FATAL_ERROR "ament_pyflakes() variable 'ament_pyflakes_BIN' must not be empty")
  endif()

  set(subcmd "${ament_pyflakes_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml\"")
  foreach(arg ${ARGN})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_pyflakes")
  set(cmd "${subcmd} > \"${CMAKE_BINARY_DIR}/ament_pyflakes/${testname}.txt\"")
  ament_add_test(
    "${testname}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
