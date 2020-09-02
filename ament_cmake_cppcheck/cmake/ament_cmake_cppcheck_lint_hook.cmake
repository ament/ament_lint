# Copyright 2015 Open Source Robotics Foundation, Inc.
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

find_package(ament_cmake_core REQUIRED)

  message(STATUS "Added test 'cppcheck' to perform static code analysis on C / C++ code")

  # Forces cppcheck to consider ament_cmake_cppcheck_LANGUAGE as the given language if defined
  set(_language "")
  if(DEFINED ament_cmake_cppcheck_LANGUAGE)
    set(_language LANGUAGE ${ament_cmake_cppcheck_LANGUAGE})
    message(STATUS "Configured cppcheck language: ${ament_cmake_cppcheck_LANGUAGE}")
  endif()

  get_directory_property(_build_targets DIRECTORY ${PROJECT_SOURCE_DIR} BUILDSYSTEM_TARGETS)
  foreach(_target ${_build_targets})
    get_target_property(_linker_language ${_target} LINKER_LANGUAGE)
    if(NOT ${linker_language} MATCHES "(C|CXX)")
      continue()
    endif()
    
    get_target_property(_target_type ${_target} TYPE)
    if(${_target_type} MATCHES "^(STATIC_LIBRARY|SHARED_LIBRARY|EXECUTABLE)$")
      set(_include_dirs $<TARGET_PROPERTY:${_target},INCLUDE_DIRECTORIES>)
      set(_definitions $<TARGET_PROPERTY:${_target},COMPILE_DEFINITIONS>)
      set(_source_files $<FILTER,$<TARGET_PROPERTY:${_target},SOURCES>,INCLUDE,"^.*[.](c|cc|cpp|cxx|h|hh|hpp|hxx)$>>")
    else()
      continue()
    endif()
    
    ament_cppcheck(
      TESTNAME ${_target}_ament_cppcheck
      ${_language}
      INCLUDE_DIRS ${ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS} ${_include_dirs}
      EXCLUDE  ${ament_cmake_cppcheck_ADDITIONAL_EXCLUDE}
      DEFINITIONS ${_definitions}
      ${_source_files}
    )
  endforeach()
