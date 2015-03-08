#
# Add a test to check the code for compliance with clang_format.
#
# :param TESTNAME: the name of the test, default: "clang_format"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_clang_format)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "clang_format")
  endif()

  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "ament_clang_format() variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()
  if(NOT ament_clang_format_BIN)
    message(FATAL_ERROR "ament_clang_format() variable 'ament_clang_format_BIN' must not be empty")
  endif()

  set(subcmd "${ament_clang_format_BIN} --xunit-file \"${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xml\"")
  foreach(arg ${ARG_UNPARSED_ARGUMENTS})
    set(subcmd "${subcmd} \"${arg}\"")
  endforeach()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_clang_format")
  set(cmd "${PYTHON_EXECUTABLE} ${subcmd} > \"${CMAKE_BINARY_DIR}/ament_clang_format/${ARG_TESTNAME}.txt\"")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endfunction()
