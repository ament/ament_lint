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
  # Get include paths for sources
  set(_include_dirs "")
  foreach(_src ${_source_files})
    get_filename_component(_src_dir ${_src} DIRECTORY)
    # Get parent directory since includes typically have the pattern 'package/source.hpp'
    get_filename_component(_parent_dir ${_src_dir} DIRECTORY)
    list(APPEND _include_dirs "${_parent_dir}")
  endforeach()
  list(REMOVE_DUPLICATES _include_dirs)
  ament_cppcheck(INCLUDE_DIRS ${_include_dirs})
endif()
