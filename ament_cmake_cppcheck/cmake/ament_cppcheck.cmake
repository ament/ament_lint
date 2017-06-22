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
# Add a test to perform static code analysis with cppcheck.
#
# :param TESTNAME: the name of the test, default: "cppcheck"
# :type TESTNAME: string
# :param LANGUAGE: the language argument for cppcheck, either 'c' or 'c++'
# :type LANGUAGE: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_cppcheck)
  cmake_parse_arguments(ARG "" "LANGUAGE;TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "cppcheck")
  endif()

  find_program(ament_cppcheck_BIN NAMES "ament_cppcheck")
  if(NOT ament_cppcheck_BIN)
    message(FATAL_ERROR "ament_cppcheck() could not find program 'ament_cppcheck'")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(cmd "${ament_cppcheck_BIN}" "--xunit-file" "${result_file}")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})
  if(ARG_LANGUAGE)
    list(APPEND cmd "--language" "${ARG_LANGUAGE}")
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_cppcheck")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cppcheck/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "cppcheck;linter"
  )
endfunction()
