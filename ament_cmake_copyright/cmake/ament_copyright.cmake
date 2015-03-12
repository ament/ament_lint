#
# Add a test to check the code for compliance with copyright.
#
# :param TESTNAME: the name of the test, default: "copyright"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_copyright)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "copyright")
  endif()

  find_program(ament_copyright_BIN NAMES "ament_copyright")
  if(NOT ament_copyright_BIN)
    message(FATAL_ERROR "ament_copyright() could not find program 'ament_copyright'")
  endif()

  set(subcmd "${ament_copyright_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xml\"")
  foreach(arg ${ARG_UNPARSED_ARGUMENTS})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_copyright")
  set(cmd "${subcmd} > \"${CMAKE_BINARY_DIR}/ament_copyright/${ARG_TESTNAME}.txt\"")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
