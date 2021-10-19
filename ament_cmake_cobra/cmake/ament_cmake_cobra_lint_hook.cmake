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

# Derived from ament_cmake_cppcheck_lint_hook.cmake

find_package(ament_cmake_core REQUIRED)

file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS
  "*.c" 
  # "*.h"
)
if(_source_files)
  message(STATUS "Added test 'cobra' to perform static code analysis on C/C++ code")

  # Get include paths for added targets
  set(_all_include_dirs "")
  set(_include_dirs "")
  if(DEFINED ament_cmake_cobra_ADDITIONAL_INCLUDE_DIRS)
    list(APPEND _all_include_dirs ${ament_cmake_cobra_ADDITIONAL_INCLUDE_DIRS})
  endif()

  # Get exclude paths for added targets
  set(_all_exclude "")
  if(DEFINED ament_cmake_cpplint_ADDITIONAL_EXCLUDE)
    list(APPEND _all_exclude ${ament_cmake_cpplint_ADDITIONAL_EXCLUDE})
  endif()

  # BUILDSYSTEM_TARGETS only supported in CMake >= 3.7
  if(NOT CMAKE_VERSION VERSION_LESS "3.7.0")
    get_directory_property(_build_targets DIRECTORY ${PROJECT_SOURCE_DIR} BUILDSYSTEM_TARGETS)
    foreach(_target ${_build_targets})
      get_target_property(_target_type ${_target} TYPE)

      # Include directories property is different for INTERFACE libraries
      if(${_target_type} STREQUAL "INTERFACE_LIBRARY")
        get_target_property(_include_dirs ${_target} INTERFACE_INCLUDE_DIRECTORIES)
      else()
        get_target_property(_include_dirs ${_target} INCLUDE_DIRECTORIES)
      endif()

      foreach(_include_dir ${_include_dirs})
        if (NOT "${_include_dirs}" STREQUAL "_include_dirs-NOTFOUND")
          list_append_unique(_all_include_dirs ${_include_dir})
        endif()
      endforeach()
    endforeach()
  endif()

  message(STATUS "Configured cobra include dirs: ${_all_include_dirs}")
  message(STATUS "Configured cobra exclude dirs and/or files: ${_all_exclude}")

  ament_cobra(TESTNAME "cobra-basic" RULESET "basic" INCLUDE_DIRS ${_all_include_dirs} EXCLUDE ${_all_exclude})
  ament_cobra(TESTNAME "cobra-cwe" RULESET "cwe" INCLUDE_DIRS ${_all_include_dirs} EXCLUDE ${_all_exclude})
  ament_cobra(TESTNAME "cobra-p10" RULESET "p10" INCLUDE_DIRS ${_all_include_dirs} EXCLUDE ${_all_exclude})
  ament_cobra(TESTNAME "cobra-jpl" RULESET "jpl" INCLUDE_DIRS ${_all_include_dirs} EXCLUDE ${_all_exclude})
  ament_cobra(TESTNAME "cobra-misra2012" RULESET "misra2012" INCLUDE_DIRS ${_all_include_dirs} EXCLUDE ${_all_exclude})
endif()
