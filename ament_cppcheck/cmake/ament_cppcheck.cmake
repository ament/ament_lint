#
# Add a test to perform static code analysis with cppcheck.
#
# :param TESTNAME: the name of the test, default: "cppcheck"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_cppcheck)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "cppcheck")
  endif()

  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "ament_cppcheck() variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()
  if(NOT ament_cppcheck_BIN)
    message(FATAL_ERROR "ament_cppcheck() variable 'ament_cppcheck_BIN' must not be empty")
  endif()

  set(subcmd "${ament_cppcheck_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xml\"")
  foreach(arg ${ARG_UNPARSED_ARGUMENTS})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_cppcheck")
  set(cmd "${PYTHON_EXECUTABLE} ${subcmd} > \"${CMAKE_BINARY_DIR}/ament_cppcheck/${ARG_TESTNAME}.txt\"")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
