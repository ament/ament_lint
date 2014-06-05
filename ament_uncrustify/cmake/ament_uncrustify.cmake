#
# Add a test to check the code for compliance with uncrustify.
#
# :param testname: the name of the test
# :type testname: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_uncrustify testname)
  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "ament_uncrustify() variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()
  if(NOT ament_uncrustify_BIN)
    message(FATAL_ERROR "ament_uncrustify() variable 'ament_uncrustify_BIN' must not be empty")
  endif()

  set(subcmd "${ament_uncrustify_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml\"")
  foreach(arg ${ARGN})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_uncrustify")
  set(cmd "${PYTHON_EXECUTABLE} ${subcmd} > \"${CMAKE_BINARY_DIR}/ament_uncrustify/${testname}.txt\"")
  ament_add_test(
    "${testname}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
