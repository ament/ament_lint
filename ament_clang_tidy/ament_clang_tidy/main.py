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
from collections import defaultdict
import copy
import json
from multiprocessing.pool import ThreadPool
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
        help='If <path> is a directory, ament_clang_tidy will recursively search it for'
             ' "compile_commands.json" files. If <path> is a file, ament_clang_tidy will'
             ' treat it as a "compile_commands.json" file')
    parser.add_argument(
        '--jobs',
        type=int,
        default=1,
        help='number of clang-tidy jobs to run in parallel')

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
        '--packages-select', nargs='*', metavar='PKG_NAME',
        help='Only process a subset of packages')
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    parser.add_argument(
        '--sarif-file',
        help='Generate a SARIF file')
    args = parser.parse_args(argv)

    if not os.path.exists(args.config_file):
        print("Could not find config file '%s'" % args.config_file,
              file=sys.stderr)
        return 1

    start_time = time.time()
    compilation_dbs = get_compilation_db_files(args.paths)

    if args.packages_select is not None:
        # Handle the case of a quoted list of space separated package names
        if len(args.packages_select) == 1:
            args.packages_select = args.packages_select[0].strip().split()
        compilation_dbs = filter_packages_select(compilation_dbs, args.packages_select)

    if not compilation_dbs:
        print('No compilation database files found', file=sys.stderr)
        return 1

    bin_names = [
        'clang-tidy',
        'clang-tidy-10',
        'clang-tidy-11',
        'clang-tidy-6.0',
    ]
    clang_tidy_bin = find_executable(bin_names)
    if not clang_tidy_bin:
        print('Could not find %s executable' %
              ' / '.join(["'%s'" % n for n in bin_names]), file=sys.stderr)
        return 1

    pool = ThreadPool(args.jobs)

    def invoke_clang_tidy(compilation_db_path):
        package_dir = os.path.dirname(compilation_db_path)
        package_name = os.path.basename(package_dir)

        with open(args.config_file, 'r') as h:
            content = h.read()
        data = yaml.safe_load(content)
        style = yaml.dump(data, default_flow_style=True, width=float('inf'))
        cmd = [clang_tidy_bin,
               '--config=%s' % style,
               '-p', package_dir]
        if args.explain_config:
            cmd.append('--explain-config')
        if args.export_fixes:
            cmd.append('--export-fixes')
            cmd.append(args.export_fixes)
        if args.fix_errors:
            cmd.append('--fix-errors')
        cmd.append('--header-filter')
        if args.header_filter:
            cmd.append(args.header_filter)
        else:
            cmd.append('include/%s/.*' % package_name)
        if args.quiet:
            cmd.append('--quiet')
        if args.system_headers:
            cmd.append('--system-headers')

        def is_gtest_source(file_name):
            if(file_name == 'gtest_main.cc' or file_name == 'gtest-all.cc'
               or file_name == 'gmock_main.cc' or file_name == 'gmock-all.cc'):
                return True
            return False

        def is_unittest_source(package, file_path):
            return ('%s/test/' % package) in file_path

        def start_subprocess(full_cmd):
            output = ''
            try:
                output = subprocess.check_output(full_cmd,
                                                 stderr=subprocess.DEVNULL).strip().decode()
            except subprocess.CalledProcessError as e:
                print('The invocation of "%s" failed with error code %d: %s' %
                      (os.path.basename(clang_tidy_bin), e.returncode, e),
                      file=sys.stderr)
            return output

        files = []
        async_outputs = []
        db = json.load(open(compilation_db_path))
        for item in db:
            # exclude gtest sources from being checked by clang-tidy
            if is_gtest_source(os.path.basename(item['file'])):
                continue

            # exclude unit test sources from being checked by clang-tidy
            # because gtest macros are problematic
            if is_unittest_source(package_name, item['file']):
                continue

            files.append(item['file'])
            full_cmd = cmd + [item['file']]
            async_outputs.append(pool.apply_async(start_subprocess, (full_cmd,)))

        output = ''
        for async_output in async_outputs:
            output += async_output.get()

        return (files, output)

    files = []
    outputs = []
    for compilation_db in compilation_dbs:
        package_dir = os.path.dirname(compilation_db)
        package_name = os.path.basename(package_dir)
        print('found compilation database for package "%s"...' % package_name)
        (source_files, output) = invoke_clang_tidy(compilation_db)
        files += source_files
        outputs.append(output)
    pool.close()
    pool.join()

    # output errors
    report = defaultdict(list)
    for filename in files:
        report[filename] = []

    # Regex explanation:
    #   \s*\^?                   : Capture many spaces, then an optional `^`. clang_tidy output isn't great.
    #   (\/home\/^[^:]*)         : Group capture. Everything from `/home/` up until a `:`.
    #   (\d+)                    : Group capture. Grabs line number.
    #   (\d+)                    : Group capture. Grabs column number.
    #   (?:warning:|error:|note:) : Non-capturing group. Matches warning, error, note.
    #   \[(.*)\]                 : Matches and captures [<rule_name>]. Ignores any messages from clang_tidy without an ending [<rule_name>].
    error_re = re.compile('\\s*\^?\/home\/([^:]*):(\\d+):(\\d+): (?:warning:|error:|note:).*\\[(.*)\\]')

    current_file = None
    new_file = None
    data = {}

    for output in outputs:
        print(output)
        for line in output.splitlines():
            # error found
            match = error_re.search(line)
            if match:
                new_file = match.group(1)
                if current_file is not None:
                    report[current_file].append(copy.deepcopy(data))
                    data.clear()
                current_file = new_file
                line_num = match.group(2)
                col_num = match.group(3)
                error_msg = find_error_message(line)
                data['line_no'] = line_num
                data['offset_in_line'] = col_num
                data['error_msg'] = error_msg
            else:
                data['code_correct_rec'] = data.get('code_correct_rec', '') + line + '\n'
        if current_file is not None:
            report[current_file].append(copy.deepcopy(data))

    elapsed_time = time.time() - start_time

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
        xml = get_xunit_content(report, testname, elapsed_time)
        path = os.path.dirname(os.path.abspath(args.xunit_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.xunit_file, 'w') as f:
            f.write(xml)

    if args.sarif_file:
        sarif = get_sarif_content(report, get_clang_tidy_version(clang_tidy_bin))
        path = os.path.dirname(os.path.abspath(args.sarif_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.sarif_file, 'w') as f:
            f.write(sarif)


def find_executable(file_names):
    paths = os.getenv('PATH').split(os.path.pathsep)
    for file_name in file_names:
        for path in paths:
            file_path = os.path.join(path, file_name)
            if os.path.isfile(file_path) and os.access(file_path, os.X_OK):
                return file_path
    return None


def get_compilation_db_files(paths):
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
                for filename in filenames:
                    if filename == 'compile_commands.json':
                        files.append(os.path.join(dirpath, filename))
        elif os.path.isfile(path):
            files.append(path)
    return [os.path.normpath(f) for f in files]


def filter_packages_select(compilation_db_paths, packages):
    def package_test(compilation_db_paths):
        package_name = os.path.basename(os.path.dirname(compilation_db_paths))
        return (package_name in packages)
    return list(filter(package_test, compilation_db_paths))


def find_error_message(data):
    return data[data.rfind(':') + 2:]


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
  errors="0"
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
                            int(error['offset_in_line']))])
                }
                if 'code_correct_rec' in data:
                    data['cdata'] += '\n'
                    data['cdata'] += data['code_correct_rec']
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
    classname="%(testname)s"/>
""" % data

    # output list of checked files
    data = {
        'escaped_files': escape(
            ''.join(['\n* %s' % r for r in sorted(map(
                os.path.relpath, report.keys()
            ))])),
    }
    xml += """  <system-out>Checked files:%(escaped_files)s</system-out>
""" % data

    xml += '</testsuite>\n'
    return xml


def get_clang_tidy_version(clang_tidy_bin):
    version = ''
    try:
        cmd_args = [clang_tidy_bin, '--version']
        output = subprocess.check_output(cmd_args).decode()
        m = re.search('.*LLVM version (.*)\n', output)
        if m:
            version = m.group(1)
    except Exception as e:
        pass

    if not version:
        # In the unlikely event that we can't get the clang tidy version, emit a warning and continue
        print(f'Failed to get the clang tidy version: {e}', file=sys.stderr)
        version = 'unknown'

    return version


def get_sarif_content(report, clang_tidy_version):
    """Transform the generic issue report format to SARIF."""

    with open('1.yaml', 'w') as file:
        yaml.dump(report, file)

    # Lay out the basic structure of the SARIF file (a single run that has 'tool',
    # 'artifacts', and 'results' entries)
    sarif = {
        'version': '2.1.0',
        '$schema': 'http://json.schemastore.org/sarif-2.1.0-rtm.5',
        'properties': {
            'comment':
            'clang-tidy output converted to SARIF by ament_clang_tidy'
        },
        'runs': [{
            'tool': {
                'driver': {
                    'name': 'clang-tidy',
                    'version': clang_tidy_version,
                    'informationUri':
                    'https://clang.llvm.org/extra/clang-tidy/',
                    'rules': []
                }
            },
            'artifacts': [],
            'results': []
        }]
    }

    # Populate rules that fired in this analysis
    rules = sarif['runs'][0]['tool']['driver']['rules']
    for filename in sorted(report.keys()):
        for error in report[filename]:
            description, rule_id = filter(
                None, re.split('\[|\]', error['error_msg']))
            new_rule = {
                'id': rule_id,
                'helpUri': 'https://clang.llvm.org/extra/clang-tidy/checks/list.html',
            }
            if new_rule not in rules:
                rules.append(new_rule)

    # Populate the artifact information (source files analyzed)
    artifacts = sarif['runs'][0]['artifacts']
    for filename in sorted(report.keys()):
        artifact = {'location': {'uri': filename}}
        artifacts.append(artifact)

    # Strip error message of rule ID
    strip_error = re.compile("(.*) \[.*]")

    # Populate the results of the analysis (issues discovered)
    results = sarif['runs'][0]['results']
    for filename in sorted(report.keys()):
        errors = report[filename]
        for error in errors:
            # Get the symbolic rule name (it's the part in brackets)
            _, rule_id = filter(None, re.split('\[|\]', error['error_msg']))

            start_line = error['line_no']
            start_column = error['offset_in_line']
            index = artifacts.index({'location': {'uri': filename}})

            error_without_rule = strip_error.match(error['error_msg'])

            results_dict = {
                'ruleId': rule_id,
                'level': 'warning',
                'kind': 'review',
                'message': {
                    'text': error_without_rule.group(1)
                },
                'locations': [{
                    'physicalLocation': {
                        'artifactLocation': {
                            'uri': filename,
                            'index': index
                        },
                        'region': {
                            'startLine': int(start_line),
                            'startColumn': int(start_column)
                        }
                    }
                }]
            }
            results.append(results_dict)

    return json.dumps(sarif, indent=2)


if __name__ == '__main__':
    sys.exit(main())
