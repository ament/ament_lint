# Copyright 2026 Open Source Robotics Foundation, Inc.
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
# Add a test to check the Python code for syntax and style compliance
# using ruff.
#
# The default configuration file for ruff is located at
# configuration/ament_ruff.toml within the ament_ruff directory
# The default configuration file can be either overridden by the
# argument 'CONFIG_FILE' or by a global variable named
# 'ament_cmake_ruff_CONFIG_FILE'
# The 'CONFIG_FILE' argument takes priority over
# 'ament_cmake_ruff_CONFIG_FILE' if both are defined
#
# :param TESTNAME: the name of the test, default: "ruff"
# :type TESTNAME: string
# :param CONFIG_FILE: the path of the configuration file for ruff to consider
# :type CONFIG_FILE: string

#
# @public
#
function(ament_ruff)
  cmake_parse_arguments(ARG "" "TESTNAME;CONFIG_FILE" "" ${ARGN})

  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "ruff")
  endif()

  find_program(ament_ruff_BIN NAMES "ament_ruff")
  if(NOT ament_ruff_BIN)
    message(FATAL_ERROR "ament_ruff() could not find program 'ament_ruff'")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(cmd "${ament_ruff_BIN}" "--xunit-file" "${result_file}")

  if(ARG_CONFIG_FILE)
    list(APPEND cmd "--config" "${ARG_CONFIG_FILE}")
  endif()

  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_ruff")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_ruff/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "ruff;linter"
  )
endfunction()
