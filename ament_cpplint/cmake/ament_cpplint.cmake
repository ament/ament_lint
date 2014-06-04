#
# Add a test to check the code for compliance with cpplint.
#
# :param testname: the name of the test
# :type testname: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_cpplint testname)
  if(NOT ament_cpplint_BIN)
    message(FATAL_ERROR "ament_cpplint() variable 'ament_cpplint_BIN' must not be empty")
  endif()

  set(subcmd "${ament_cpplint_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml\"")
  foreach(arg ${ARGN})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_cpplint")
  # cpplint only works with Python 2
  set(cmd "python2 ${subcmd} > \"${CMAKE_BINARY_DIR}/ament_cpplint/${testname}.txt\"")
  ament_add_test(
    "${testname}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
