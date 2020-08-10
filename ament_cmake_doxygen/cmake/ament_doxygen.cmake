# Copyright 2020 Open Source Robotics Foundation, Inc.
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
# Add a test to check the code for compliance with cpplint.
#
# :param TESTNAME: the name of the test, default: "cpplint"
# :type TESTNAME: string
# :param FILTERS: list of category filters to apply
# :type FILTERS: list of strings
# :param MAX_LINE_LENGTH: override the maximum line length,
#   the default is defined in ament_cpplint
# :type MAX_LINE_LENGTH: integer
# :param ROOT: override the --root option of cpplint
# :type ROOT: string
# :param TIMEOUT: the test timeout in seconds, default: 120
# :type TIMEOUT: integer
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_doxygen)
  cmake_parse_arguments(ARG "" "TESTNAME;DOXYFILE" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "doxygen")
  endif()

  find_program(ament_doxygen_BIN NAMES "ament_doxygen")
  if(NOT ament_doxygen_BIN)
    message(FATAL_ERROR "ament_doxygen() could not find program 'ament_doxygen'")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(cmd "${ament_doxygen_BIN}")#"--xunit-file" "${result_file}")

  if(ARG_DOXYFILE)
    list(APPEND cmd "${ARG_DOXYFILE}")
  endif()

  message("ARG_DOXYFILE " ${ARG_DOXYFILE})
  message("ARG_UNPARSED_ARGUMENTS " ${ARG_UNPARSED_ARGUMENTS})

  # list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})
  if(NOT ARG_TIMEOUT)
    set(ARG_TIMEOUT 120)
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_doxygen")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_doxygen/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT "${ARG_TIMEOUT}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "doxygen;linter"
  )
endfunction()
