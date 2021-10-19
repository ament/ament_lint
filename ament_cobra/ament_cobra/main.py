#!/usr/bin/env python3

# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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
from collections import defaultdict
import glob
import multiprocessing
import os
import re
from shutil import which
import subprocess
import sys
import time
from xml.etree import ElementTree
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr


def get_cobra_version(cobra_bin):
    version_cmd = [cobra_bin, '-V']
    output = subprocess.check_output(version_cmd)
    # Expecting something like: b'Version 2.14 - 19 December 2016\n'
    output = output.decode().strip()
    tokens = output.split()
    if len(tokens) != 6:
        raise RuntimeError("unexpected cobra version string '{}'".format(output))
    return tokens[1]


def main(argv=sys.argv[1:]):
    extensions = ['c', 'cc', 'cpp', 'cxx']
    rulesets = [ 'basic', 'cwe', 'p10', 'jpl', 'misra2012' ]

    parser = argparse.ArgumentParser(
        description='Analyze source code using the cobra static analyzer.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help='Files and/or directories to be checked. Directories are searched recursively for '
             'files ending in one of %s.' %
             ', '.join(["'.%s'" % e for e in extensions]))
    parser.add_argument(
        '--include_dirs',
        nargs='*',
        help="Include directories for C/C++ files being checked."
             "Each directory is passed to cobra as '-I<include_dir>'")
    parser.add_argument(
        '--exclude', default=[],
        nargs='*',
        help='Exclude C/C++ files from being checked.')
    parser.add_argument(
        '--ruleset',
        default='basic',
        help="The cobra rule set to use to analyze the code: basic, cwe, p10, jpl, or misra2012.")
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    parser.add_argument(
        '--cobra-version',
        action='store_true',
        help='Get the cobra version, print it, and then exit')

    args = parser.parse_args(argv)

    cobra_bin = find_executable('cobra')
    if not cobra_bin:
        print("Could not find 'cobra' executable", file=sys.stderr)
        return 1

    cobra_version = get_cobra_version(cobra_bin)
    if args.cobra_version:
        print(cobra_version)
        return 0

    groups = get_file_groups(args.paths, extensions, args.exclude)
    if not groups:
        print('No files found', file=sys.stderr)
        return 1

    cmd = [cobra_bin, '-C++', '-comments', '-scrub' ]

    if args.ruleset in rulesets:
        cmd.extend(['-f', args.ruleset ])
    else:
        print(f'Invalid ruleset specified: {args.ruleset}')
        return 1

    for include_dir in (args.include_dirs or []):
        cmd.extend(['-I'+include_dir])

    # Initialize the output report
    report = defaultdict(list)

    # Invoke cobra on each group
    for root in sorted(groups.keys()):
        arguments = list(cmd)
        files = groups[root]

        # Even though we use a defaultdict, explicitly add known files so they are listed
        for filename in files:
            report[filename] = []

        arguments.extend(files)

        if args.xunit_file:
            start_time = time.time()

        try:
            p = subprocess.Popen(arguments, stdout=subprocess.PIPE)
            cmd_output = p.communicate()[0]
        except subprocess.CalledProcessError as e:
            print("The invocation of 'cobra' failed with error code %d: %s" %
                  (e.returncode, e), file=sys.stderr)
            return 1

        test_failures = []
        try:
            lines = cmd_output.decode("utf-8").split('\n')
            for line in lines:
              print(line)

              # TODO(mjeronimo): Parse the cobra output
              #
              # if line.startswith('cobra:'):
              #   [ cobra_name, filename, lineno, message ] = line.split(':', 4)

            # Add one example failure for now. TODO(mjeronimo): remove when parsing has been implemented
            failure = {}
            filename = "/home/michael/src/ros2/src/ros2/rcutils/src/logging.c"
            failure['filename'] = filename
            failure['line'] = 895
            failure['severity'] = 'error'
            failure['msg'] = "Macro argument not enclosed in parentheses"
            report[filename].append(failure)
        except ElementTree.ParseError as e:
            print('Invalid XML in cobra output: %s' % str(e),
                  file=sys.stderr)
            return 1

        for failure in test_failures:
            for key in report.keys():
                if os.path.samefile(key, filename):
                    filename = key
                    break

            # In the case where relative and absolute paths are mixed for paths and
            # include_dirs cobra might return duplicate results
            if failure not in report[filename]:
                report[filename].append(failure)

    # Output a summary
    error_count = sum(len(r) for r in report.values())
    if not error_count:
        print('No problems found')
        rc = 0
    else:
        print('%d errors' % error_count, file=sys.stderr)
        rc = 1

    # Generate the xunit output file
    if args.xunit_file:
        write_xunit_file(args.xunit_file, report, time.time() - start_time)

    return rc


def find_executable(file_name, additional_paths=None):
    path = None
    if additional_paths:
        path = os.getenv('PATH', os.defpath)
        path += os.path.pathsep + os.path.pathsep.join(additional_paths)
    return which(file_name, path=path)


def get_files(paths, extensions):
    files = []
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                if 'AMENT_IGNORE' in dirnames + filenames:
                    dirnames[:] = []
                    continue
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    _, ext = os.path.splitext(filename)
                    if ext in ['.%s' % e for e in extensions]:
                        files.append(os.path.join(dirpath, filename))
        if os.path.isfile(path):
            files.append(path)
    return [os.path.normpath(f) for f in files]


def get_file_groups(paths, extensions, exclude_patterns):
    excludes = []
    for exclude_pattern in exclude_patterns:
        excludes.extend(glob.glob(exclude_pattern))
    excludes = {os.path.realpath(x) for x in excludes}

    # dict mapping root path to files
    groups = {}
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                if 'AMENT_IGNORE' in dirnames + filenames:
                    dirnames[:] = []
                    continue
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    _, ext = os.path.splitext(filename)
                    if ext in ('.%s' % e for e in extensions):
                        filepath = os.path.join(dirpath, filename)
                        if os.path.realpath(filepath) not in excludes:
                            append_file_to_group(groups, filepath)

        if os.path.isfile(path):
            if os.path.realpath(path) not in excludes:
                append_file_to_group(groups, path)

    return groups


def get_xunit_content(report, testname, elapsed, skip=None):
    test_count = sum(max(len(r), 1) for r in report.values())
    error_count = sum(len(r) for r in report.values())
    data = {
        'testname': testname,
        'test_count': test_count,
        'error_count': error_count,
        'time': '%.3f' % round(elapsed, 3),
        'skip': test_count if skip else 0,
    }
    xml = """<?xml version="1.0" encoding="UTF-8"?>
<testsuite
  name="%(testname)s"
  tests="%(test_count)d"
  errors="0"
  failures="%(error_count)d"
  time="%(time)s"
  skipped="%(skip)d"
>
""" % data

    for filename in sorted(report.keys()):
        errors = report[filename]

        if skip:
            data = {
              'quoted_name': quoteattr(filename),
              'testname': testname,
              'quoted_message': quoteattr(''),
              'skip': skip,
            }
            xml += """  <testcase
    name=%(quoted_name)s
    classname="%(testname)s"
  >
    <skipped type="skip" message=%(quoted_message)s>
      ![CDATA[Test Skipped due to %(skip)s]]
    </skipped>
  </testcase>
""" % data
        elif errors:
            # report each cobra error as a failing testcase
            for error in errors:
                data = {
                    'quoted_name': quoteattr(
                        #'%s: %s (%s:%s)' % (
                        '%s: (%s:%s)' % (
                            error['severity'], # error['id'],
                            filename, error['line'])),
                    'testname': testname,
                    'quoted_message': quoteattr(error['msg']),
                }
                xml += """  <testcase
    name=%(quoted_name)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s/>
  </testcase>
""" % data

        else:
            # if there are no cpplint errors report a single successful test
            data = {
                'quoted_location': quoteattr(filename),
                'testname': testname,
            }
            xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"/>
""" % data

    # output list of checked files
    if skip:
        data = {
            'skip': skip,
        }
        xml += """  <system-err>Tests Skipped due to %(skip)s</system-err>
""" % data
    else:
        data = {
            'escaped_files': escape(
                ''.join(['\n* %s' % r for r in sorted(report.keys())])
            ),
        }
        xml += """  <system-out>Checked files:%(escaped_files)s</system-out>
""" % data

    xml += '</testsuite>\n'
    return xml


def append_file_to_group(groups, path):
    path = os.path.abspath(path)

    root = ''

    # try to determine root from path
    base_path = os.path.dirname(path)
    # find longest subpath which ends with one of the following subfolder names
    subfolder_names = ['include', 'src', 'test']
    matches = [
        re.search(
            '^(.+%s%s)%s' %
            (re.escape(os.sep), re.escape(subfolder_name), re.escape(os.sep)), path)
        for subfolder_name in subfolder_names]
    match_groups = [match.group(1) for match in matches if match]
    if match_groups:
        match_groups = [{'group_len': len(x), 'group': x} for x in match_groups]
        sorted_groups = sorted(match_groups, key=lambda k: k['group_len'])
        base_path = sorted_groups[-1]['group']
        root = base_path

    # try to find repository root
    repo_root = None
    p = path
    while p and repo_root is None:
        # abort if root is reached
        if os.path.dirname(p) == p:
            break
        p = os.path.dirname(p)
        for marker in ['.git', '.hg', '.svn']:
            if os.path.exists(os.path.join(p, marker)):
                repo_root = p
                break

    # compute relative --root argument
    if repo_root and repo_root > base_path:
        root = os.path.relpath(base_path, repo_root)

    # add the path to the appropriate group
    if root not in groups:
        groups[root] = []
    groups[root].append(path)


def write_xunit_file(xunit_file, report, duration, skip=None):
    folder_name = os.path.basename(os.path.dirname(xunit_file))
    file_name = os.path.basename(xunit_file)
    suffix = '.xml'
    if file_name.endswith(suffix):
        file_name = file_name[0:-len(suffix)]
        suffix = '.xunit'
        if file_name.endswith(suffix):
            file_name = file_name[0:-len(suffix)]
    testname = '%s.%s' % (folder_name, file_name)

    xml = get_xunit_content(report, testname, duration, skip)
    path = os.path.dirname(os.path.abspath(xunit_file))
    if not os.path.exists(path):
        os.makedirs(path)
    with open(xunit_file, 'w') as f:
        f.write(xml)


if __name__ == '__main__':
    sys.exit(main())
