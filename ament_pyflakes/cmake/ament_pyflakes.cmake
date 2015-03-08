#
# Add a test to check the code for compliance with pyflakes.
#
# :param TESTNAME: the name of the test, default: "pyflakes"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_pyflakes)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "pyflakes")
  endif()

  if(NOT ament_pyflakes_BIN)
    message(FATAL_ERROR "ament_pyflakes() variable 'ament_pyflakes_BIN' must not be empty")
  endif()

  set(subcmd "${ament_pyflakes_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xml\"")
  foreach(arg ${ARG_UNPARSED_ARGUMENTS})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_pyflakes")
  set(cmd "${subcmd} > \"${CMAKE_BINARY_DIR}/ament_pyflakes/${ARG_TESTNAME}.txt\"")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
