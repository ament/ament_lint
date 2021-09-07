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


def find_uno_executable():
    additional_paths = None
    if os.name == 'nt':
        # search in location where uno is installed via chocolatey
        program_files_32 = os.environ.get('ProgramFiles(x86)', 'C:\\Program Files (x86)')
        additional_paths = [os.path.join(program_files_32, 'uno')]
    return find_executable('uno', additional_paths=additional_paths)


def get_uno_version(uno_bin):
    version_cmd = [uno_bin, '-V']
    output = subprocess.check_output(version_cmd)
    # Expecting something like: b'Version 2.14 - 19 December 2016\n'
    output = output.decode().strip()
    tokens = output.split()
    if len(tokens) != 6:
        raise RuntimeError("unexpected uno version string '{}'".format(output))
    return tokens[1]


def main(argv=sys.argv[1:]):
    extensions = ['c']

    parser = argparse.ArgumentParser(
        description='Perform static code analysis using uno.',
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
             "Each directory is passed to uno as '-I<include_dir>'")

    parser.add_argument(
        '--exclude',
        nargs='*',
        help="Exclude C/C++ files from being checked."
             "Each file is passed to uno as '--suppress=*:<file>'")

    # Not using a file handle directly in order to prevent leaving
    # an empty file when something fails early
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')

    # Option to just get the uno version and print that
    parser.add_argument(
        '--uno-version',
        action='store_true',
        help='Get the uno version, print it, and then exit')

    args = parser.parse_args(argv)

    uno_bin = find_uno_executable()
    if not uno_bin:
        print("Could not find 'uno' executable", file=sys.stderr)
        return 1

    uno_version = get_uno_version(uno_bin)
    if args.uno_version:
        print(uno_version)
        return 0

    if args.xunit_file:
        start_time = time.time()

    files = get_files(args.paths, extensions)
    if not files:
        print('No files found', file=sys.stderr)
        return 1

    # invoke uno
    cmd = [uno_bin,
    # -a -e -r -t -u -D__builtin_va_list= -D__w64=
           # '-a',
           # '-e',
            '-l',
           # '-n',
           # '-r',
           # '-t',
           # '-u',
           # '-v',
           '-D__builtin_va_list=',
           '-D__w64=']

    for include_dir in (args.include_dirs or []):
        cmd.extend(['-I'+include_dir])

    for exclude in (args.exclude or []):
        cmd.extend(['--suppress=*:' + exclude])

    cmd.extend(files)

    print(' '.join(cmd))

    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        cmd_output = p.communicate()[0]
    except subprocess.CalledProcessError as e:
        print("The invocation of 'uno' failed with error code %d: %s" %
              (e.returncode, e), file=sys.stderr)
        return 1

    errors = []
    try:
        lines = cmd_output.decode("utf-8").split('\n')
        for line in lines:
          print(f'line: {line}')
          if line.startswith('uno:'):
            #if 'array indexing error' in line:
            #  line = line.replace('13 array indexing error', '13: array indexing error')
            #  line = line.replace('array-size: ', 'array-size ')

            [ uno_name, filename, lineno, message ] = line.split(':', 4)

            error = {}
            error['filename'] = filename.strip()
            error['line'] = lineno.strip()
            error['severity'] = 'error'
            error['msg'] = message.strip()

            errors.append(error)
    except ElementTree.ParseError as e:
        print('Invalid XML in uno output: %s' % str(e),
              file=sys.stderr)
        return 1

    # Output errors
    report = defaultdict(list)

    # Even though we use a defaultdict, explicity add known files so they are listed
    for filename in files:
        report[filename] = []

    for error in errors:
        for key in report.keys():
            if os.path.samefile(key, filename):
                filename = key
                break

        # In the case where relative and absolute paths are mixed for paths and
        # include_dirs uno might return duplicate results
        if error not in report[filename]:
            report[filename].append(error)

    # output summary
    error_count = sum(len(r) for r in report.values())
    if not error_count:
        print('No problems found')
        rc = 0
    else:
        print('%d errors' % error_count, file=sys.stderr)
        rc = 1

    # Generate xunit file
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
            # report each uno error as a failing testcase
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
