# Copyright 2021 Open Source Robotics Foundation, Inc.
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
# Add a test to perform static code analysis with cobra.
#
# The include dirs for cobra to consider can either be set by the function
# parameter 'INCLUDE_DIRS` or by a global variable called
# 'ament_cmake_cobra_ADDITIONAL_INCLUDE_DIRS'.
#
# :param TESTNAME: the name of the test, default: "cobra"
# :type TESTNAME: string
# :param INCLUDE_DIRS: an optional list of include paths for cobra
# :type INCLUDE_DIRS: list
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_cobra)
  cmake_parse_arguments(ARG "" "TESTNAME" "INCLUDE_DIRS" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "cobra")
  endif()

  find_program(ament_cobra_BIN NAMES "ament_cobra")
  if(NOT ament_cobra_BIN)
    message(FATAL_ERROR "ament_cobra() could not find program 'ament_cobra'")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(cmd "${ament_cobra_BIN}" "--xunit-file" "${result_file}")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})

  if(ARG_INCLUDE_DIRS)
    list(APPEND cmd "--include_dirs" "${ARG_INCLUDE_DIRS}")
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_cobra")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    TIMEOUT 302
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cobra/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "cobra;linter"
  )
endfunction()
