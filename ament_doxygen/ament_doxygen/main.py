#!/usr/bin/env python3

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

import argparse
import os
import pathlib
import re
import shutil
import subprocess
import sys


def is_file_or_directory_to_exclude(path_to_file, excludes):
    if not os.path.isdir(path_to_file):
        path, file_name = os.path.split(path_to_file)
    else:
        path = path_to_file
        file_name = ''
    path_to_check = pathlib.Path(path)

    def check_path_parents(parents, exclude):
        for p in parents:
            if p.match(exclude):
                return True
        return False

    for excl in excludes:
        if (
            excl == file_name or
            path_to_check.match(excl) or
            check_path_parents(path_to_check.parents, excl)
        ):
            return True
    return False

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Check undocumented code using doxyfile.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--excludes', default=[],
        nargs='*',
        help='Exclude directories or C/C++ files to be checked.')
    parser.add_argument('doxyfile', default='Doxyfile')
    args = parser.parse_args()

    doxygen_bin = find_executable('doxygen')
    if not doxygen_bin:
        print("Could not find 'doxygen_bin' executable", file=sys.stderr)
        return 1

    number_of_warnings = invoke_doxygen(doxygen_bin, args.doxyfile, args.excludes)

    print('Total errors found: %d' % number_of_warnings, file=sys.stderr)

    return 1 if number_of_warnings else 0


def invoke_doxygen(doxygen_bin, doxyfile, excludes):
    dir_path = os.path.dirname(os.path.realpath(doxyfile))
    cmd = [doxygen_bin, doxyfile]
    number_of_warnings = 0
    try:
        output = subprocess.check_output(cmd, cwd=dir_path, stderr=subprocess.STDOUT)
        output = output.decode().strip()
        lines = output.split('\n')
        for line in lines:
            if 'warning:' in line:

                warning_file = line.split(':')[0]
                if not is_file_or_directory_to_exclude(warning_file, excludes):
                    number_of_warnings += 1
                    print(line)
    except subprocess.CalledProcessError as e:
        if e.output:
            print(e.output.decode(), file=sys.stderr)
        print("The invocation of 'doxyfile' failed with error code %d: %s" %
              (e.returncode, e), file=sys.stderr)
        return None
    return number_of_warnings

def find_executable(file_name, additional_paths=None):
    path = None
    if additional_paths:
        path = os.getenv('PATH', os.defpath)
        path += os.path.pathsep + os.path.pathsep.join(additional_paths)
    return shutil.which(file_name, path=path)


if __name__ == '__main__':
    sys.exit(main())
