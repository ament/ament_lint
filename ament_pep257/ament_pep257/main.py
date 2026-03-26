#!/usr/bin/env python3

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


import argparse
from importlib.util import find_spec
import os
import shutil
import sys
import time
from typing import Literal
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr

ruff_installed = False
pydocstyle_installed = False

if shutil.which('ruff'):
    ruff_installed = True
else:
    ruff_installed = False

if find_spec('pydocstyle'):
    pydocstyle_installed = True
    import pydocstyle
    _conventions = set(pydocstyle.conventions.keys())
    _conventions.add('ament')
else:
    pydocstyle_installed = False
    _conventions = {'pep257', 'numpy', 'google', 'ament'}

_ament_ignore = [
    'D100',
    'D101',
    'D102',
    'D103',
    'D104',
    'D105',
    'D106',
    'D107',
    'D203',
    'D212',
    'D404',
]


def main(argv: list[str] = sys.argv[1:]) -> Literal[0, 1]:
    parser = argparse.ArgumentParser(
        description='Check docstrings against the style conventions in PEP 257.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    err_code_group = parser.add_mutually_exclusive_group()
    err_code_group.add_argument(
        '--ignore',
        nargs='+',
        default=[],
        help='Choose the list of error codes for pydocstyle NOT to check for.')
    err_code_group.add_argument(
        '--select',
        nargs='+',
        default=[],
        help='Choose the basic list of error codes for pydocstyle to check for.'
    )
    err_code_group.add_argument(
        '--convention',
        choices=_conventions,
        default='ament',
        help=(
            f'Choose a preset list of error codes. Valid options are {_conventions}.'
            f'The "ament" convention is defined as --ignore {_ament_ignore}.'
        ),
    )
    parser.add_argument(
        '--add-ignore',
        nargs='+',
        default=[],
        help='Ignore an extra error code, removing it from the list set by --(select/ignore)')
    parser.add_argument(
        '--add-select',
        nargs='+',
        default=[],
        help='Check an extra error code, adding it to the list set by --(select/ignore).'
    )
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help='The files or directories to check. For directories, files ending '
             "in '.py' will be considered.")
    parser.add_argument(
        '--exclude',
        metavar='filename',
        nargs='*',
        default=[],
        dest='excludes',
        help='The filenames to exclude.')
    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    args = parser.parse_args(argv)

    if args.xunit_file:
        start_time = time.time()

    args.ignore = ','.join(args.ignore)
    args.select = ','.join(args.select)
    args.add_select = ','.join(args.add_select)
    args.add_ignore = ','.join(args.add_ignore)
    if not (args.ignore or args.select) and args.convention == 'ament':
        args.ignore = ','.join(_ament_ignore)

    excludes = [os.path.abspath(e) for e in args.excludes]

    if ruff_installed:
        from ament_pep257.ruff_impl import generate_ruff_report
        report = generate_ruff_report(args.paths, excludes, args.ignore, args.select,
                                      args.convention, args.add_ignore, args.add_select)
    elif pydocstyle_installed:
        from ament_pep257.pydocstyle_impl import generate_pep257_report
        report = generate_pep257_report(args.paths, excludes, args.ignore, args.select,
                                        args.convention, args.add_ignore, args.add_select)
    else:
        print('Neither ruff or pydocstyle installed')
        return 1
    error_count = sum(len(r[1]) for r in report)

    # print summary
    if not error_count:
        print('No problems found')
        rc = 0
    else:
        print('%d errors' % error_count, file=sys.stderr)
        rc = 1

    # generate xunit file
    if args.xunit_file:
        folder_name = os.path.basename(os.path.dirname(args.xunit_file))
        file_name = os.path.basename(args.xunit_file)
        suffix = '.xml'
        if file_name.endswith(suffix):
            file_name = file_name[0:-len(suffix)]
            suffix = '.xunit'
            if file_name.endswith(suffix):
                file_name = file_name[0:-len(suffix)]
        testname = '%s.%s' % (folder_name, file_name)

        xml = get_xunit_content(report, testname, time.time() - start_time)
        path = os.path.dirname(os.path.abspath(args.xunit_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.xunit_file, 'w') as f:
            f.write(xml)

    return rc


def _filename_in_excludes(filename, excludes):
    absname = os.path.abspath(filename)
    return any(os.path.commonpath([absname, e]) == e for e in excludes)


def get_xunit_content(report, testname, elapsed):
    test_count = sum(max(len(r[1]), 1) for r in report)
    error_count = sum(len(r[1]) for r in report)
    data = {
        'testname': testname,
        'test_count': test_count,
        'error_count': error_count,
        'time': '%.3f' % round(elapsed, 3),
    }
    xml = """<?xml version="1.0" encoding="UTF-8"?>
<testsuite
  name="%(testname)s"
  tests="%(test_count)d"
  errors="0"
  failures="%(error_count)d"
  time="%(time)s"
>
""" % data

    for (filename, errors) in report:
        if errors:
            # report each error as a failing testcase
            for error in errors:
                data = {
                    'quoted_location': quoteattr(
                        '%s (%s:%s)' % (
                            error['category'], filename, str(error['linenumber']))),
                    'testname': testname,
                    'quoted_message': quoteattr(error['message']),
                }
                xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s/>
  </testcase>
""" % data

        else:
            # if there are no lint_cmake errors report a single successful test
            data = {
                'quoted_location': quoteattr(filename),
                'testname': testname,
            }
            xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"/>
""" % data

    # output list of checked files
    data = {
        'escaped_files': escape(''.join(['\n* %s' % r[0] for r in report])),
    }
    xml += """  <system-out>Checked files:%(escaped_files)s</system-out>
""" % data

    xml += '</testsuite>\n'
    return xml


if __name__ == '__main__':
    sys.exit(main())
