# Copyright 2015-2018 Open Source Robotics Foundation, Inc.
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

# Forces ament_xmllint to consider ament_cmake_xmllint_EXTENSIONS as the given extensions if defined
set(_extensions "xml")
if(DEFINED ament_cmake_xmllint_EXTENSIONS)
  string(REGEX REPLACE "[ \\*\\.,]+" ";" _extensions ${ament_cmake_xmllint_EXTENSIONS})
  message(STATUS "Configured xmllint extensions: ${_extensions}")
endif()

# Make sure all extensions start with '*.' or add it if not
foreach(_extension ${_extensions})
  string(REGEX MATCH "^\\*?\\.?(.+)$" _bare_extension ${_extension})
  list(APPEND _glob_extensions "*.${_bare_extension}")
endforeach()

message(DEBUG "Globbing for xml files with extensions: ${_glob_extensions}")

file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS ${_glob_extensions})
if(_source_files)
  message(DEBUG "Found files with extensions: ${_source_files}")
  message(STATUS "Added test 'xmllint' to check XML markup files")
  ament_xmllint(EXTENSIONS ${_extensions})
else()
  message(DEBUG "No files with extensions: ${_extensions} found, skipping test 'xmllint'")
endif()
