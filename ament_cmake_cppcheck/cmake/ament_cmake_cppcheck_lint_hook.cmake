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

file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS
  "*.c"
  "*.cc"
  "*.cpp"
  "*.cxx"
  "*.h"
  "*.hh"
  "*.hpp"
  "*.hxx"
)
if(_source_files)
  message(STATUS "Added test 'cppcheck' to perform static code analysis on C / C++ code")

  # Get include paths for added targets
  set(_all_include_dirs "")
  # BUILDSYSTEM_TARGETS only supported in CMake >= 3.7
  if(NOT CMAKE_VERSION VERSION_LESS "3.7.0")
    get_directory_property(_build_targets DIRECTORY ${CMAKE_SOURCE_DIR} BUILDSYSTEM_TARGETS)
    foreach(_target ${_build_targets})
      get_property(_include_dirs
        TARGET ${_target}
        PROPERTY INCLUDE_DIRECTORIES
      )

      # Only append include directories that are from the package being tested
      # This accomplishes two things:
      #     1. Reduces execution time (less include directories to search)
      #     2. cppcheck will not check for errors in external packages
      foreach(_include_dir ${_include_dirs})
        string(REGEX MATCH "^${CMAKE_SOURCE_DIR}.*" _is_match ${_include_dir})
        if(_is_match)
          list_append_unique(_all_include_dirs ${_include_dir})
        endif()
      endforeach()
    endforeach()
  endif()

  ament_cppcheck(INCLUDE_DIRS ${_all_include_dirs})
endif()
