#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
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
import copy
import os
import re
import subprocess
import sys
import time

from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr

import yaml


def main(argv=sys.argv[1:]):
    config_file = os.path.join(
        os.path.dirname(__file__), 'configuration', '.clang-tidy')
    extensions = ['c', 'cc', 'cpp', 'cxx', 'h', 'hh', 'hpp', 'hxx']

    parser = argparse.ArgumentParser(
        description='Check code style using clang_tidy.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--config',
        metavar='path',
        default=config_file,
        dest='config_file',
        help='The config file')
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help='The files or directories to check. For directories files ending '
             'in %s will be considered.' %
             ', '.join(["'.%s'" % e for e in extensions]))

    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument(
        '--explain-config',
        action='store_true',
        help='Explain the enabled checks')
    parser.add_argument(
        '--export-fixes',
        help='Generate a DAT file of recorded fixes')
    parser.add_argument(
        '--fix-errors',
        action='store_true',
        help='Fix the suggested changes')
    parser.add_argument(
        '--header-filter',
        help='Accepts a regex and displays errors from the specified non-system headers')
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Suppresses printing statistics about ignored warnings '
             'and warnings treated as errors')
    parser.add_argument(
        '--system-headers',
        action='store_true',
        help='Displays errors from all system headers')
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    args = parser.parse_args(argv)

    if not os.path.exists(args.config_file):
        print("Could not find config file '%s'" % args.config_file,
              file=sys.stderr)
        return 1

    if args.xunit_file:
        start_time = time.time()

    files = get_files(args.paths, extensions)
    if not files:
        print('No files found', file=sys.stderr)
        return 1

    bin_names = [
        'clang-tidy',
        'clang-tidy-6.0',
    ]
    clang_tidy_bin = find_executable(bin_names)
    if not clang_tidy_bin:
        print('Could not find %s executable' %
              ' / '.join(["'%s'" % n for n in bin_names]), file=sys.stderr)
        return 1

    # invoke clang_tidy
    with open(args.config_file, 'r') as h:
        content = h.read()
    data = yaml.safe_load(content)
    style = yaml.dump(data, default_flow_style=True, width=float('inf'))
    cmd = [clang_tidy_bin,
           '--config=%s' % style]
    if args.explain_config:
        cmd.append('--explain-config')
    if args.export_fixes:
        cmd.append('--export-fixes')
        cmd.append(args.export_fixes)
    if args.fix_errors:
        cmd.append('--fix-errors')
    if args.header_filter:
        cmd.append('--header-filter')
        cmd.append(args.header_filter)
    if args.quiet:
        cmd.append('--quiet')
    if args.system_headers:
        cmd.append('--system-headers')
    cmd.extend(files)
    cmd.append('--')
    try:
        output = subprocess.check_output(cmd).strip().decode()
        print(output)
    except subprocess.CalledProcessError as e:
        print("The invocation of '%s' failed with error code %d: %s" %
              (os.path.basename(clang_tidy_bin), e.returncode, e),
              file=sys.stderr)
        return 1

    # output errors
    report = {}
    complete_filenames = []

    for filename in files:
        report[filename] = []
        complete_filename = os.path.join(
            os.getcwd(), filename)
        complete_filenames.append(complete_filename)

    error_re = re.compile(r'\[.*?\]')
    file_pairs = dict(zip(complete_filenames, files))

    current_file = None
    new_file = None
    data = {}

    for line in output.splitlines():
        # error found
        if line[0] == '/' and error_re.search(line):
            for filename in complete_filenames:
                if filename in line:
                    new_file = file_pairs[filename]
                    if current_file is not None:
                        report[current_file].append(copy.deepcopy(data))
                        data.clear()
                    current_file = new_file
            error_msg = find_error_message(line)
            line_num, col_num = find_line_and_col_num(line)
            data['line_no'] = line_num
            data['offset_in_line'] = col_num
            data['error_msg'] = error_msg
        else:
            data['code_correct_rec'] = data.get('code_correct_rec', '') + line + '\n'
    if current_file is not None:
        report[current_file].append(copy.deepcopy(data))

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
    return


def find_executable(file_names):
    paths = os.getenv('PATH').split(os.path.pathsep)
    for file_name in file_names:
        for path in paths:
            file_path = os.path.join(path, file_name)
            if os.path.isfile(file_path) and os.access(file_path, os.X_OK):
                return file_path
    return None


def get_files(paths, extensions):
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
                    _, ext = os.path.splitext(filename)
                    if ext in ('.%s' % e for e in extensions):
                        files.append(os.path.join(dirpath, filename))
        if os.path.isfile(path):
            files.append(path)
    return [os.path.normpath(f) for f in files]


def find_error_message(data):
    return data[data.rfind(':') + 2:]


def find_line_and_col_num(data):
    first_col = data.find(':')
    second_col = data.find(':', first_col + 1)
    third_col = data.find(':', second_col + 1)
    return data[first_col + 1:second_col], data[second_col + 1:third_col]


def get_xunit_content(report, testname, elapsed):
    test_count = sum(max(len(r), 1) for r in report.values())
    error_count = sum(len(r) for r in report.values())
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
  failures="%(error_count)d"
  time="%(time)s"
>
""" % data

    for filename in sorted(report.keys()):
        errors = report[filename]

        if errors:
            # report each replacement as a failing testcase
            for error in errors:
                data = {
                    'quoted_location': quoteattr(
                        '%s:%d:%d' % (
                            filename, int(error['line_no']),
                            int(error['offset_in_line']))),
                    'testname': testname,
                    'quoted_message': quoteattr(
                        '%s' %
                        error['error_msg']),
                    'cdata': '\n'.join([
                        '%s:%d:%d' % (
                            filename, int(error['line_no']),
                            int(error['offset_in_line'])),
                        error['code_correct_rec'],
                    ]),
                }
                xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s><![CDATA[%(cdata)s]]></failure>
  </testcase>
""" % data

        else:
            # if there are no errors report a single successful test
            data = {
                'quoted_location': quoteattr(filename),
                'testname': testname,
            }
            xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"
    status="No problems found"/>
""" % data

    # output list of checked files
    data = {
        'escaped_files': escape(''.join(['\n* %s' % r
                                         for r in sorted(report.keys())])),
    }
    xml += """  <system-out>Checked files:%(escaped_files)s</system-out>
""" % data

    xml += '</testsuite>\n'
    return xml


if __name__ == '__main__':
    sys.exit(main())
