#
# Add a test to check the code for compliance with copyright.
#
# :param testname: the name of the test
# :type testname: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_copyright testname)
  if(NOT ament_copyright_BIN)
    message(FATAL_ERROR "ament_copyright() variable 'ament_copyright_BIN' must not be empty")
  endif()

  set(subcmd "${ament_copyright_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml\"")
  foreach(arg ${ARGN})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_copyright")
  # copyright only works with Python 2
  set(cmd "python2 ${subcmd} > \"${CMAKE_BINARY_DIR}/ament_copyright/${testname}.txt\"")
  ament_add_test(
    "${testname}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
