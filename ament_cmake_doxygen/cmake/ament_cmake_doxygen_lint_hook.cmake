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

file(GLOB_RECURSE _doxyfile FOLLOW_SYMLINKS
  "Doxyfile"
)
if (_doxyfile)
  message(STATUS "Added test 'doxygen' to check C / C++ code without documentation")
  ament_doxygen(DOXYFILE ${_doxyfile})
else ()
  # TODO(ahcorde): Logic to generate the Doxyfile

  file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS
    "*.h"
    "*.hh"
    "*.hpp"
    "*.hxx"
    "*.h.em"
    "*.hh.em"
    "*.hpp.em"
    "*.hxx.em"
    "*.h.in"
    "*.hh.in"
    "*.hpp.in"
    "*.hxx.in"
  )
  if (_source_files)
    message(STATUS "No Doxyfile found, generating one.")
  endif()
endif()
