# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
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
# Add a test to check YAML files with yamllint.
#
# :param TESTNAME: the name of the test, default: "yamllint"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_yamllint)
  cmake_parse_arguments(ARG "" "MAX_LINE_LENGTH;TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "yamllint")
  endif()

  find_program(ament_yamllint_BIN NAMES "ament_yamllint")
  if(NOT ament_yamllint_BIN)
    message(FATAL_ERROR "ament_yamllint() could not find program 'ament_yamllint'")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(cmd "${ament_yamllint_BIN}" "--xunit-file" "${result_file}")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})

  find_program(yamllint_BIN NAMES "yamllint")

  if(NOT yamllint_BIN)
    if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
      message(WARNING "WARNING: 'yamllint' not found, skipping yamllint test creation")
      return()
    endif()
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_yamllint")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_yamllint/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "yamllint;linter"
  )
  if(NOT yamllint_BIN)
    set_tests_properties(
      "${ARG_TESTNAME}"
      PROPERTIES
      DISABLED TRUE
    )
  endif()
endfunction()
