#
# Add a test to check the Python code for compliance with PEP8.
#
# :param TESTNAME: the name of the test, default: "pep8"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_pep8)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "pep8")
  endif()

  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "ament_pep8() variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()
  if(NOT ament_pep8_BIN)
    message(FATAL_ERROR "ament_pep8() variable 'ament_pep8_BIN' must not be empty")
  endif()

  set(subcmd "${ament_pep8_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xml\"")
  foreach(arg ${ARG_UNPARSED_ARGUMENTS})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_pep8")
  set(cmd "${PYTHON_EXECUTABLE} ${subcmd} > \"${CMAKE_BINARY_DIR}/ament_pep8/${ARG_TESTNAME}.txt\"")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
