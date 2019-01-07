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
  # BUILDSYSTEM_TARGETS only supported in cmake >= 3.7
  if(CMAKE_VERSION VERSION_EQUAL "3.7.0" OR CMAKE_VERSION VERSION_GREATER "3.7.0")
    foreach(target ${BUILDSYSTEM_TARGETS})
      get_property(_include_dirs
        TARGET ${target}
        PROPERTY INCLUDE_DIRECTORIES
      )
      list(APPEND _all_include_dirs "${_include_dirs}")
    endforeach()
  endif()

  ament_cppcheck(INCLUDE_DIRS "${_all_include_dirs}")
endif()
