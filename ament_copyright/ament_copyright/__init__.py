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

import pkg_resources


COPYRIGHT_GROUP = 'ament_copyright.copyright_name'
LICENSE_GROUP = 'ament_copyright.license'

SOURCE_FILETYPE = 1
CONTRIBUTING_FILENAME = 'CONTRIBUTING.md'
CONTRIBUTING_FILETYPE = 2
LICENSE_FILENAME = 'LICENSE'
LICENSE_FILETYPE = 3

ALL_FILETYPES = {
    SOURCE_FILETYPE: None,
    CONTRIBUTING_FILETYPE: CONTRIBUTING_FILENAME,
    LICENSE_FILETYPE: LICENSE_FILENAME,
}

UNKNOWN_IDENTIFIER = '<unknown>'


def get_copyright_names():
    names = {}
    for entry_point in pkg_resources.iter_entry_points(
            group=COPYRIGHT_GROUP):
        assert entry_point.name != UNKNOWN_IDENTIFIER, \
            "Invalid entry point name '%s'" % entry_point.name
        name = entry_point.load()
        names[entry_point.name] = name
    return names


def get_licenses():
    licenses = {}
    for entry_point in pkg_resources.iter_entry_points(
            group=LICENSE_GROUP):
        assert entry_point.name != UNKNOWN_IDENTIFIER, \
            "Invalid entry point name '%s'" % entry_point.name
        licenses[entry_point.name] = entry_point.load()
    return licenses
