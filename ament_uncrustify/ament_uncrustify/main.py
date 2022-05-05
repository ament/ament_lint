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
from configparser import ConfigParser
import difflib
import filecmp
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr


def get_uncrustify_version(uncrustify_bin):
    version_cmd = [uncrustify_bin, '--version']
    output = subprocess.check_output(version_cmd)
    # expecting something like b'Uncrustify_d-0.72.0-dirty'
    output = output.decode().strip()
    if not output.startswith('Uncrustify'):
        raise RuntimeError("unexpected uncrustify version string '{}'".format(output))
    return output


def main(argv=sys.argv[1:]):
    config_file = os.path.join(
        os.path.dirname(__file__),
        'configuration', 'ament_code_style.cfg')

    c_extensions = ['c', 'cc', 'h', 'hh']
    cpp_extensions = ['cpp', 'cxx', 'hpp', 'hxx']

    parser = argparse.ArgumentParser(
        description='Check code style using uncrustify.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-c',
        metavar='CFG',
        default=config_file,
        dest='config_file',
        help='The config file')
    parser.add_argument(
        '--linelength', metavar='N', type=int,
        help='The maximum line length (default: specified in the config file)')
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help='The files or directories to check. For directories files ending '
             'in %s will be considered.' %
             ', '.join([
                 "'.%s'" % e for e in sorted(c_extensions + cpp_extensions)]))
    parser.add_argument(
        '--exclude',
        nargs='*',
        default=[],
        help='Exclude specific file names and directory names from the check')
    parser.add_argument(
        '--language',
        choices=['C', 'C++', 'CPP'],
        help="Passed to uncrustify as '-l <language>' to force a specific "
             'language rather then choosing one based on file extension')
    parser.add_argument(
        '--reformat',
        action='store_true',
        help='Reformat the files in place')
    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    parser.add_argument(
        '--sarif-file',
        help='Generate a SARIF file')
    args = parser.parse_args(argv)

    if not os.path.exists(args.config_file):
        print("Could not config file '%s'" % args.config_file, file=sys.stderr)
        return 1

    temp_config = None
    temp_path = None
    try:
        if args.linelength is not None:
            # check if different from config file
            config = ConfigParser()
            with open(args.config_file, 'r') as h:
                config_str = h.read()
            config.read_string('[DEFAULT]\n' + config_str)
            code_width = config['DEFAULT']['code_width']
            code_width = int(re.split('[ \t#]', code_width, maxsplit=1)[0])
            if args.linelength != code_width:
                # generate temporary config file with custom line length
                temp_config_fd, args.config_file = tempfile.mkstemp(prefix='uncrustify_')
                temp_config = os.fdopen(temp_config_fd, 'w')
                temp_config.write(config_str + '\ncode_width=%d' % args.linelength)
                temp_config.close()

        start_time = time.time()

        # replace language set to 'C++' with 'CPP' to be more consistent with uncrustify
        if args.language == 'C++':
            language_ = 'CPP'
        else:
            language_ = args.language

        files_by_language = get_files(
            args.paths, {'C': c_extensions, 'CPP': cpp_extensions},
            excludes=args.exclude, language=language_)
        if not files_by_language:
            print('No files found', file=sys.stderr)
            return 1

        uncrustify_bin = find_executable('uncrustify')
        if not uncrustify_bin:
            print("Could not find 'uncrustify' executable", file=sys.stderr)
            return 1

        report = []
        temp_path = tempfile.mkdtemp(prefix='uncrustify_')
        suffix = '.uncrustify'

        # invoke uncrustify on all files
        all_input_files = []
        all_output_files = []
        for language, input_files in files_by_language.items():
            all_input_files += input_files
            output_files = invoke_uncrustify(
                uncrustify_bin, input_files, args, language, temp_path, suffix)
            if output_files is None:
                return 1
            all_output_files += output_files

        # compute diff
        for filename, modified_filename in sorted(zip(all_input_files, all_output_files)):
            with open(filename, 'r', encoding='utf-8') as original_file:
                with open(modified_filename, 'r', encoding='utf-8') as modified_file:
                    diff_lines = list(difflib.unified_diff(
                        original_file.readlines(), modified_file.readlines(),
                        fromfile=filename, tofile=filename + suffix,
                        n=0))
                    report.append((filename, diff_lines))
            if args.reformat:
                # overwrite original with reformatted file
                with(open(filename, 'wb')) as original_file:
                    with open(modified_filename, 'rb') as modified_file:
                        original_file.write(modified_file.read())
    finally:
        if temp_config:
            os.remove(args.config_file)
        if temp_path and os.path.exists(temp_path):
            shutil.rmtree(temp_path)

    # output diffs
    for (filename, diff_lines) in report:
        if diff_lines:
            if not args.reformat:
                print("Code style divergence in file '%s':" % filename,
                      file=sys.stderr)
                print('', file=sys.stderr)
                for line in diff_lines:
                    print(line.rstrip('\r\n'), file=sys.stderr)
                print('', file=sys.stderr)
            else:
                print("Code style divergence in file '%s': reformatted file" %
                      filename)
        else:
            print("No code style divergence in file '%s'" % filename)
            if not args.reformat:
                print('')

    elapsed_time = time.time() - start_time

    # output summary
    error_count = sum([1 if r[1] else 0 for r in report])
    if not error_count:
        print('No problems found')
        rc = 0
    else:
        print('%d files with code style divergence' % error_count,
              file=sys.stderr)
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
        testname = f'{folder_name}.{file_name}'

        xml = get_xunit_content(report, testname, elapsed_time)
        path = os.path.dirname(os.path.abspath(args.xunit_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.xunit_file, 'w') as f:
            f.write(xml)

    # generate a SARIF file
    if args.sarif_file:
        folder_name = os.path.basename(os.path.dirname(args.sarif_file))
        file_name = os.path.basename(args.sarif_file)
        suffix = '.sarif'
        if file_name.endswith(suffix):
            file_name = file_name[0:-len(suffix)]
        testname = f'{folder_name}.{file_name}'

        sarif = get_sarif_content(report,
                                  testname,
                                  elapsed_time,
                                  get_uncrustify_version(uncrustify_bin))
        path = os.path.dirname(os.path.abspath(args.sarif_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.sarif_file, 'w') as f:
            f.write(sarif)

    return rc


def find_executable(file_name, additional_paths=None):
    path = None
    if additional_paths:
        path = os.getenv('PATH', os.defpath)
        path += os.path.pathsep + os.path.pathsep.join(additional_paths)
    return shutil.which(file_name, path=path)


def get_files(paths, extension_types, excludes=[], language=None):
    extensions_with_dot_to_language = {
        f'.{extension}': language or ext_language
        for ext_language, extensions in extension_types.items()
        for extension in extensions
    }
    files = defaultdict(list)
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                if 'AMENT_IGNORE' in dirnames + filenames:
                    dirnames[:] = []
                    continue
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                # ignore excluded folders
                dirnames[:] = [d for d in dirnames if d not in excludes]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    if filename in excludes:
                        continue
                    _, ext = os.path.splitext(filename)
                    language = extensions_with_dot_to_language.get(ext, None)
                    if language is not None:
                        files[language].append(
                            os.path.normpath(os.path.join(dirpath, filename)))
        if os.path.isfile(path):
            _, ext = os.path.splitext(path)
            language = extensions_with_dot_to_language.get(ext, None)
            files[language].append(os.path.normpath(path))
    return files


def invoke_uncrustify(
    uncrustify_bin, files, args, language, temp_path, suffix
):
    if not files:
        return []

    # invoke uncrustify on all files
    input_files = [os.path.abspath(f) for f in files]

    # on Windows uncrustify fails to concatenate
    # the absolute prefix path with the absolute input files
    # https://github.com/bengardner/uncrustify/issues/364
    cwd = None
    if os.name == 'nt':
        cwd = os.path.commonprefix(input_files)
        if not os.path.isdir(cwd):
            cwd = os.path.dirname(cwd)
            assert os.path.isdir(cwd), \
                'Could not determine common prefix of input files'
        input_files = [os.path.relpath(f, start=cwd) for f in input_files]

    try:
        cmd = [uncrustify_bin, '-c', args.config_file]
        if language:
            cmd += ['-l', language]
        cmd += [
            '--prefix', temp_path,
            '--suffix', suffix]
        cmd.extend(input_files)
        subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
        if e.output:
            print(e.output.decode(), file=sys.stderr)
        print("The invocation of 'uncrustify' failed with error code %d: %s" %
              (e.returncode, e), file=sys.stderr)
        return None

    if cwd:
        # input files are relative
        # prepend temp path, append suffix
        output_files = [
            os.path.join(temp_path, f + suffix) for f in input_files]
    else:
        # input files are absolute
        # remove leading slash, prepend temp path, append suffix
        output_files = [
            os.path.join(
                temp_path,
                os.sep.join(f.split(os.sep)[1:]) + suffix
            ) for f in input_files
        ]

    uncrustified_files = output_files
    i = 1
    while True:
        # identify files which have changed since the latest uncrustify run
        changed_files = []
        for input_filename, output_filename in zip(
                input_files, uncrustified_files):
            if cwd and not os.path.isabs(input_filename):
                input_filename = os.path.join(cwd, input_filename)
            if not filecmp.cmp(input_filename, output_filename):
                if output_filename == input_filename + suffix:
                    # for repeated invocations
                    # replace the previous uncrustified file
                    dst_filename = input_filename
                else:
                    # after first invocation remove suffix
                    # otherwise uncrustify behaves different
                    output_filename_without_suffix = \
                        output_filename[:-len(suffix)]
                    dst_filename = output_filename_without_suffix
                if os.path.exists(dst_filename):
                    # on Windows os.rename doesn't replace an existing dst
                    os.remove(dst_filename)
                os.rename(output_filename, dst_filename)
                changed_files.append(dst_filename)
        if not changed_files:
            break
        # reinvoke uncrustify for previously changed files
        input_files = changed_files
        try:
            cmd = [uncrustify_bin, '-c', args.config_file]
            if language:
                cmd += ['-l', language]
            cmd += ['--suffix', suffix]
            cmd.extend(input_files)
            subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            if e.output:
                print(e.output, file=sys.stderr)
            print("The invocation of 'uncrustify' failed with error code %d: "
                  '%s' % (e.returncode, e), file=sys.stderr)
            return None

        uncrustified_files = [f + suffix for f in input_files]
        i += 1
        if i >= 5:
            print("'uncrustify' did not settle on a final result even after "
                  '%d invocations' % i, file=sys.stderr)
            return None

    return output_files


def get_xunit_content(report, testname, elapsed):
    test_count = len(report)
    error_count = sum([1 if r[1] else 0 for r in report])
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
  errors="0"
  time="%(time)s"
>
""" % data

    for (filename, diff_lines) in report:

        if diff_lines:
            # report any diff as a failing testcase
            data = {
                'quoted_location': quoteattr(filename),
                'testname': testname,
                'quoted_message': quoteattr(
                    'Diff with %d lines' % len(diff_lines)
                ),
                'cdata': ''.join(diff_lines),
            }
            xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s><![CDATA[%(cdata)s]]></failure>
  </testcase>
""" % data

        else:
            # if there is no diff report a single successful test
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


def get_sarif_content(report, testname, elapsed, uncrustify_version):
    test_count = len(report)
    error_count = sum([1 if r[1] else 0 for r in report])

    # Lay out the basic structure of the SARIF file (a single run that has 'tool',
    # 'artifacts', and 'results' entries)
    sarif = {
        'version': '2.1.0',
        '$schema': 'http://json.schemastore.org/sarif-2.1.0-rtm.5',
        'properties': {
            'comment': 'uncrustify output converted to SARIF by ament_uncrustify',
            'test_name': testname,
            'test_count': test_count,
            'error_count': error_count,
            'execution_time': '%.3f' % round(elapsed, 3),
        },
        'runs': [{
            'tool': {
                'driver': {
                    'name': 'uncrustify',
                    'version': uncrustify_version,
                    'informationUri': 'http://uncrustify.sourceforge.net/',
                    'rules': []
                }
            },
            'artifacts': [],
            'results': []
        }]
    }

    # For convenience, get entries to the 'rules', 'artifacts', and 'results' sections
    rules = sarif['runs'][0]['tool']['driver']['rules']
    artifacts = sarif['runs'][0]['artifacts']
    results = sarif['runs'][0]['results']

    # There is only one rule that this utility checks
    rule_id = 'non-compliant-code-formatting'
    message = 'Source code does not comply with code formatting standards'
    rules.append({
        'id': rule_id,
        'shortDescription': {
            'text': message
        },
    })

    for (filename, diff_lines) in report:
        # Populate the artifact information (source files analyzed)
        artifacts.append({'location': {'uri': filename}})

        # Process any associated error/warning info associated with this file
        if diff_lines:
            # Some data to keep track of for each violation
            starting_line = None
            ending_line = None
            span = None
            code_segment = []

            # Split the output into separate formatting violations
            for line in diff_lines:
                # Skip the diff lines containing the filenames
                if line.startswith('---') or line.startswith('+++'):
                    continue

                # The '@@' begins a formatting issue
                if line.startswith('@@'):
                    # If we've been collecting an issue (true for all but the first time)
                    if code_segment:
                        # Populate the results with this code formatting issue
                        results.append({
                            'ruleId': rule_id,
                            'level': 'error',
                            'message': {
                                'text': message,
                            },
                            'locations': [{
                                'physicalLocation': {
                                    'artifactLocation': {
                                        'uri': filename,
                                        'index': artifacts.index({'location': {'uri': filename}}),
                                    },
                                    'region': {
                                        'startLine': int(starting_line),
                                        'endLine': int(ending_line),
                                        'message': {'text': '\n'.join(code_segment)}
                                    },
                                }
                            }],
                        })
                        code_segment = [line]

                    # Update the info for the next issue
                    m = re.search('@@ -([0-9]+)(,([0-9]+))?', line)
                    starting_line = m.group(1)
                    # If the issue is multi-line, it will have a second number
                    span = m.group(3) if m.group(3) else 1
                    ending_line = str(int(starting_line) + int(span) - 1)
                else:
                    # Continue to accumulate the code in this segment
                    code_segment.append(line)

    return json.dumps(sarif, indent=2)


if __name__ == '__main__':
    sys.exit(main())
