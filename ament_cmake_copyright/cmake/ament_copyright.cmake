# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Add a test to check the code for compliance with copyright.
#
# :param TESTNAME: the name of the test, default: "copyright"
# :type TESTNAME: string
# :param TIMEOUT: the test timeout in seconds, default: 120
# :type TIMEOUT: integer
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_copyright)
  cmake_parse_arguments(ARG "" "TESTNAME;TIMEOUT" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "copyright")
  endif()
  if(NOT ARG_TIMEOUT)
    set(ARG_TIMEOUT 120)
  endif()

  # https://cmake.org/cmake/help/latest/prop_dir/TESTS.html
  get_directory_property(_declared_tests TESTS)
  if(DEFINED AMENT_LINT_AUTO_SKIP_PREEXISTING_TESTS)
    if((AMENT_LINT_AUTO_SKIP_PREEXISTING_TESTS) AND (${ARG_TESTNAME} IN_LIST _declared_tests))
      message(VERBOSE "skipping test '${ARG_TESTNAME}' as it has already been added")
      return()
    endif()
  endif()

  find_program(ament_copyright_BIN NAMES "ament_copyright")
  if(NOT ament_copyright_BIN)
    message(FATAL_ERROR "ament_copyright() could not find program 'ament_copyright'")
  endif()

  set(xunit_result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(sarif_result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.sarif")
  set(cmd "${ament_copyright_BIN}" "--xunit-file" "${xunit_result_file}" "--sarif-file" "${sarif_result_file}")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_copyright")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_copyright/${ARG_TESTNAME}.txt"
    RESULT_FILE "${xunit_result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT "${ARG_TIMEOUT}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "copyright;linter"
  )
endfunction()
