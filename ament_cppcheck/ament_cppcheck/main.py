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
import multiprocessing
import os
from shutil import which
import subprocess
import sys
import time
from xml.etree import ElementTree
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr


def find_cppcheck_executable():
    additional_paths = None
    if os.name == 'nt':
        # search in location where cppcheck is installed via chocolatey
        program_files_32 = os.environ.get('ProgramFiles(x86)', 'C:\\Program Files (x86)')
        additional_paths = [os.path.join(program_files_32, 'Cppcheck')]
    return find_executable('cppcheck', additional_paths=additional_paths)


def get_cppcheck_version(cppcheck_bin):
    version_cmd = [cppcheck_bin, '--version']
    output = subprocess.check_output(version_cmd)
    # expecting something like b'Cppcheck 1.88\n'
    output = output.decode().strip()
    tokens = output.split()
    if len(tokens) != 2:
        raise RuntimeError("unexpected cppcheck version string '{}'".format(output))
    return tokens[1]


def main(argv=sys.argv[1:]):
    extensions = ['c', 'cc', 'cpp', 'cxx', 'h', 'hh', 'hpp', 'hxx']

    parser = argparse.ArgumentParser(
        description='Perform static code analysis using cppcheck.',
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
             "Each directory is passed to cppcheck as '-I <include_dir>'")
    parser.add_argument(
        '--language',
        help="Passed to cppcheck as '--language=<language>', and it forces cppcheck to consider "
             "as the given language ('c' or 'c++').")
    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    # option to just get the cppcheck version and print that
    parser.add_argument(
        '--cppcheck-version',
        action='store_true',
        help='Get the cppcheck version, print it, and then exit.')
    args = parser.parse_args(argv)

    cppcheck_bin = find_cppcheck_executable()
    if not cppcheck_bin:
        print("Could not find 'cppcheck' executable", file=sys.stderr)
        return 1

    cppcheck_version = get_cppcheck_version(cppcheck_bin)

    if args.cppcheck_version:
        print(cppcheck_version)
        return 0

    if args.xunit_file:
        start_time = time.time()

    files = get_files(args.paths, extensions)
    if not files:
        print('No files found', file=sys.stderr)
        return 1

    # try to determine the number of CPU cores
    jobs = None
    try:
        jobs = multiprocessing.cpu_count()
    except NotImplementedError:
        # the number of cores cannot be determined, do not extend args
        pass

    # detect cppcheck 1.88 which caused issues
    if 'AMENT_CPPCHECK_ALLOW_1_88' not in os.environ:
        if cppcheck_version == '1.88':
            print(
                'cppcheck 1.88 has known performance issues and therefore will not be used, '
                'set the AMENT_CPPCHECK_ALLOW_1_88 environment variable to override this.',
                file=sys.stderr,
            )

            if args.xunit_file:
                report = {input_file: [] for input_file in files}
                write_xunit_file(
                    args.xunit_file, report, time.time() - start_time,
                    skip='cppcheck 1.88 performance issues'
                )
                return 0

            return 188

    # invoke cppcheck
    cmd = [cppcheck_bin,
           '-f',
           '--inline-suppr',
           '-q',
           '-rp',
           '--xml',
           '--xml-version=2']
    if args.language:
        cmd.extend(['--language={0}'.format(args.language)])
    for include_dir in (args.include_dirs or []):
        cmd.extend(['-I', include_dir])
    if jobs:
        cmd.extend(['-j', '%d' % jobs])
    cmd.extend(files)
    try:
        p = subprocess.Popen(cmd, stderr=subprocess.PIPE)
        xml = p.communicate()[1]
    except subprocess.CalledProcessError as e:
        print("The invocation of 'cppcheck' failed with error code %d: %s" %
              (e.returncode, e), file=sys.stderr)
        return 1

    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError as e:
        print('Invalid XML in cppcheck output: %s' % str(e),
              file=sys.stderr)
        return 1

    # output errors
    report = {}
    for filename in files:
        report[filename] = []
    for error in root.find('errors'):
        location = error.find('location')
        filename = location.get('file')
        data = {
            'line': int(location.get('line')),
            'id': error.get('id'),
            'severity': error.get('severity'),
            'msg': error.get('verbose'),
        }
        for key in report.keys():
            if os.path.samefile(key, filename):
                filename = key
                break
        # in the case where relative and absolute paths are mixed for paths and
        # include_dirs cppcheck might return duplicate results
        if data not in report[filename]:
            report[filename].append(data)

            data = dict(data)
            data['filename'] = filename
            print('[%(filename)s:%(line)d]: (%(severity)s: %(id)s) %(msg)s' % data,
                file=sys.stderr)

    # output summary
    error_count = sum(len(r) for r in report.values())
    if not error_count:
        print('No problems found')
        rc = 0
    else:
        print('%d errors' % error_count, file=sys.stderr)
        rc = 1

    # generate xunit file
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
                if 'AMENT_IGNORE' in filenames:
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
  failures="%(error_count)d"
  time="%(time)s"
  skip="%(skip)d"
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
    status="notrun"
  >
    <skipped type="skip" message=%(quoted_message)s>
      ![CDATA[Test Skipped due to %(skip)s]]
    </skipped>
  </testcase>
""" % data
        elif errors:
            # report each cppcheck error as a failing testcase
            for error in errors:
                data = {
                    'quoted_name': quoteattr(
                        '%s: %s (%s:%d)' % (
                            error['severity'], error['id'],
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
    classname="%(testname)s"
    status="No problems found"/>
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
