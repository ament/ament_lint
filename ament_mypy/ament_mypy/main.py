#!/usr/bin/env python3

# Copyright 2019 Canonical, Ltd.
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
from distutils.version import LooseVersion
import os
import re
import sys
import time
from typing import List, Match, Optional, Tuple
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr

import mypy.api

def main(argv: List[str] = sys.argv[1:]) -> int:
    parser = argparse.ArgumentParser(
        description='Check code using mypy',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '--config',
        metavar='path',
        dest='config_file',
        help='The config file'
    )
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help='The files or directories to check. For directories files ending '
             "in '.py' will be considered."
    )
    parser.add_argument(
        '--exclude',
        metavar='filename',
        nargs='*',
        dest='excludes',
        help='The filenames to exclude.'
    )
    parser.add_argument(
        '--cache-dir',
        metavar='cache',
        default=os.devnull,
        dest='cache_dir',
        help='The location mypy will place its cache in. Defaults to system '
             'null device'
    )

    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file'
    )
    args = parser.parse_args(argv)

    if args.xunit_file:
        start_time = time.time()

    filenames = get_files(args.paths)
    if args.excludes:
        filenames = [f for f in filenames
            if os.path.basename(f) not in args.excludes]
    if not filenames:
        print('No files found', file=sys.stderr)
        return 1

    if args.config_file and not os.path.exists(args.config_file):
        print("Could not find config file '{}'".format(args.config_file),
            file=sys.stderr
        )
        return 1

    normal_report, error_messages, exit_code = generate_mypy_report(
        args.config_file,
        filenames,
        args.cache_dir
    )

    if error_messages:
        print('mypy error encountered', file=sys.stderr)
        print(error_messages, file=sys.stderr)
        return exit_code

    errors_parsed = get_errors(normal_report)

    print('\n{} files checked'.format(len(filenames)))
    if not normal_report:
        print('No errors found')
    else:
        print('{} errors'.format(len(errors_parsed)))


    print(normal_report)

    print('\nChecked files:')
    print(''.join(['\n* {}'.format(f) for f in filenames]))

    # generate xunit file
    if args.xunit_file:
        folder_name = os.path.basename(os.path.dirname(args.xunit_file))
        file_name = os.path.basename(args.xunit_file)
        suffix = '.xml'
        if file_name.endswith(suffix):
            file_name = file_name[:-len(suffix)]
            suffix = '.xunit'
            if file_name.endswith(suffix):
                file_name = file_name[:-len(suffix)]
        testname = '{}.{}'.format(folder_name, file_name)

        xml = get_xunit_content(errors_parsed, testname, filenames, time.time() - start_time)
        path = os.path.dirname(os.path.abspath(args.xunit_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.xunit_file, 'w') as f:
            f.write(xml)

    return exit_code


def generate_mypy_report(config_file: str,
                         paths: List[str],
                         cache_dir: str) -> Tuple[str, str, int]:
    mypy_argv = []
    mypy_argv.append('--cache-dir')
    mypy_argv.append(str(cache_dir))
    if cache_dir == os.devnull:
        mypy_argv.append('--no-incremental')
    if config_file is not None:
        mypy_argv.append('--config-file')
        mypy_argv.append(str(config_file))
    if not os.environ.get('MYPYPATH') and os.environ.get('PYTHONPATH') and os.environ.get('COLCON_PREFIX_PATH'):
        # Filter PYTHONPATHs by colcon prefixes
        python_paths = os.environ['PYTHONPATH'].split(':')
        colcon_paths = os.environ['COLCON_PREFIX_PATH'].split(':')
        intersecting_paths = []
        for python_path in python_paths:
            for colcon_path in colcon_paths:
                # The colcon prefix is pointing at the install space, but ideally we could extract
                # both install-space and build-space paths from PYTHONPATH. There doesn't seem to be
                # a way to extract these programatically, so we'll assume that the defaults are being
                # used and the install- and build-space share a parent directory. This assumption will
                # not be valid if the `--build-base` or `--install-base` options were used for
                # `colcon build`.
                if python_path and python_path.startswith(os.path.dirname(colcon_path)):
                    intersecting_paths.append(python_path)
                    break

        os.environ['MYPYPATH'] = ':'.join(intersecting_paths)
    mypy_argv.append('--show-error-context')
    mypy_argv.append('--show-column-numbers')
    mypy_argv.append('--show-traceback')
    mypy_argv += paths
    res = mypy.api.run(mypy_argv)  # type: Tuple[str, str, int]
    return res


def get_xunit_content(errors: List[Match], testname: str, filenames: List[str], elapsed: float) -> str:
    xml = """<?xml version="1.0" encoding="UTF-8"?>
<testsuite
  name="{test_name:s}"
  tests="{test_count:d}"
  failures="{error_count:d}"
  time="{time:s}"
>
""".format(
        test_name = testname,
        test_count = max(len(errors), 1),
        error_count = len(errors),
        time = '{:.3f}'.format(round(elapsed, 3))
    )

    if len(errors):
        # report each mypy error/warning as a failing testcase
        for error in errors:
            xml += """  <testcase
    name={quoted_name}
    classname="{test_name}"
  >
      <failure message={quoted_message}/>
  </testcase>
""".format(
        quoted_name = quoteattr(
                        '{0[type]} ({0[filename]}'.format(error) +
                        (':{0[lineno]}:{0[colno]})'.format(error)
                        if error.group('filename') else ')')),
        test_name = testname,
        quoted_message = quoteattr('{0[msg]}'.format(error) +
                    (':\n{0[lineno]}'.format(error) if error.group('lineno')
                        else ''))
        )
    else:
        # if there are no mypy problems report a single successful test
        xml += """  <testcase
    name="mypy"
    classname="{test_name}"
    status="No problems found"/>
""".format(test_name = testname)

    # output list of checked files
    xml += """  <system-out>Checked files:{escaped_files}</system-out>
""".format(escaped_files = escape(''.join(['\n* %s' % f for f in filenames])))

    xml += '</testsuite>\n'
    return xml


def get_files(paths: List[str]) -> List[str]:
    files = []
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                if 'AMENT_IGNORE' in filenames:
                    dirnames[:] = []
                    continue
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    if filename.endswith('.py'):
                        files.append(os.path.join(dirpath, filename))
        if os.path.isfile(path):
            files.append(path)
    return [os.path.normpath(f) for f in files]


def get_errors(report_string: str) -> List[Match]:
    return list(re.finditer(r"^(?P<filename>[^:]+):(?P<lineno>\d+):(?P<colno>\d+):\ (?P<type>error|warning):\ (?P<msg>.*)$", report_string, re.MULTILINE))


if __name__ == '__main__':
    sys.exit(main())
