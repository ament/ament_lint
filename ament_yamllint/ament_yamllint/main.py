#!/usr/bin/env python3

# Copyright 2026 Open Source Robotics Foundation, Inc.
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
import errno
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr

import yaml


def main(argv=sys.argv[1:]):
    config_file = os.path.join(
        os.path.dirname(__file__), 'configuration', 'yamllint.yaml'
    )

    extensions = ['yaml', 'yml']

    parser = argparse.ArgumentParser(
        description='Check YAML style using YAMLlint.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '-c',
        '--config',
        metavar='CFG',
        default=config_file,
        dest='config_file',
        help='The config file',
    )
    parser.add_argument(
        '--linelength',
        metavar='N',
        type=int,
        help='The maximum line length (default: specified in the config file)',
    )
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help='The files or directories to check. For each directory only files ending '
        'in %s will be considered.' % ', '.join(["'.%s'" % e for e in extensions]),
    )
    parser.add_argument(
        '--exclude',
        nargs='*',
        default=[],
        help='Exclude specific file names and directory names from the check',
    )
    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument('--xunit-file', help='Generate a xunit compliant XML file')
    args = parser.parse_args(argv)

    if not os.path.exists(args.config_file):
        print("Could not find config file '%s'" % args.config_file, file=sys.stderr)
        return 1

    files = get_files(args.paths, extensions, args.exclude)
    if not files:
        print('No files found', file=sys.stderr)
        return 1

    if args.xunit_file:
        start_time = time.time()

    with open(args.config_file) as f:
        yamllint_config = yaml.safe_load(f)

    assert isinstance(yamllint_config, dict), 'Invalid configuration file'
    yamllint_config['yaml-files'] = ['*.%s' % (ext,) for ext in extensions]
    if args.linelength is not None:
        if not isinstance(yamllint_config.get('rules'), dict):
            yamllint_config['rules'] = {}
        if not isinstance(yamllint_config['rules'].get('line-length'), dict):
            yamllint_config['rules']['line-length'] = {}
        yamllint_config['rules']['line-length']['max'] = args.linelength

    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='yamllint_', delete=True
    ) as temp_config:
        yaml.dump(yamllint_config, temp_config)
        temp_config.flush()
        report = invoke_yamllint(files, config_file=temp_config.name)

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

    if any(report.values()):
        return 1

    print('No problems found, checked %d files' % (len(report),))
    return 0


def get_files(paths, extensions, excludes=None):
    if excludes is None:
        excludes = []

    files = []
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                IGNORE_MARKERS = {'AMENT_IGNORE', 'COLCON_IGNORE'}
                if any(m in dirnames + filenames for m in IGNORE_MARKERS):
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
                    if ext not in ['.%s' % e for e in extensions]:
                        continue
                    files.append(os.path.join(dirpath, filename))
        elif os.path.isfile(path):
            files.append(path)
    return [os.path.normpath(f) for f in files]


def find_executable(file_name, additional_paths=None):
    path = None
    if additional_paths:
        path = os.getenv('PATH', os.defpath)
        path += os.path.pathsep + os.path.pathsep.join(additional_paths)
    return shutil.which(file_name, path=path)


_YAMLLINT_PARSABLE_RE = re.compile(r'^(.*?):(\d+):(\d+): \[(.+)\] (.*) \((.+)\)$')


def _parse_yamllint_parsable(stdout: str) -> list[dict]:
    # Parse yamllint -f parsable output into list of issue dicts with filename.
    issues = []
    for line in stdout.splitlines():
        m = _YAMLLINT_PARSABLE_RE.match(line)
        if not m:
            raise ValueError(f'Failed to parse yamllint output: {line !r}')
        issues.append(
            {
                'file': m.group(1),
                'line': int(m.group(2)),
                'col': int(m.group(3)),
                'severity': m.group(4),
                'msg': m.group(5),
                'id': m.group(6),
            }
        )
    return issues


def invoke_yamllint(files, config_file=None):
    yamllint_bin = find_executable('yamllint')
    if not yamllint_bin:
        raise FileNotFoundError(errno.ENOENT, 'Could not find executable', 'yamllint')

    cmd = [yamllint_bin, '-s', '-f', 'parsable']
    if config_file:
        cmd += ['-c', config_file]
    cmd += list(files)

    norm_to_original = {os.path.normpath(f): f for f in files}
    report = {f: [] for f in files}

    completed = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    stdout = (completed.stdout or b'').decode(errors='replace')
    stderr = (completed.stderr or b'').decode(errors='replace')
    returncode = completed.returncode

    if stderr:
        print(stderr, end='', file=sys.stderr)

    # 1. if returncode is 0, no issues found
    if returncode == 0:
        return report

    # 2. if returncode is not 0, fail
    # if there is stdout, try to parse it
    if stdout:
        print(stdout, end='')
        try:
            issues = _parse_yamllint_parsable(stdout)
            # successfully parsed yamllint output, now populate the report
            for it in issues:
                raw_file = it.pop('file')
                key = norm_to_original.get(os.path.normpath(raw_file))
                if key is None:
                    raise ValueError(f'Got failures for unknown file: {raw_file}')
                report[key].append(it)
            return report
        except ValueError:
            # if there was a stdout but we couldn't parse it, treat it as a real error
            pass

    # 3. rc!=0 and no parsable output, treat as error
    raise subprocess.CalledProcessError(
        returncode, cmd, output=completed.stdout, stderr=completed.stderr
    )


def get_xunit_content(report, testname, elapsed):
    test_count = sum(max(len(r), 1) for r in report.values())
    error_count = sum(len(r) for r in report.values())
    data = {
        'testname': testname,
        'test_count': test_count,
        'error_count': error_count,
        'time': '%.3f' % round(elapsed, 3),
    }
    xml = (
        """<?xml version='1.0' encoding='UTF-8'?>
<testsuite
  name='%(testname)s'
  tests='%(test_count)d'
  errors='0'
  failures='%(error_count)d'
  time='%(time)s'
  skipped='0'
>
"""
        % data
    )

    for filename in sorted(report.keys()):
        errors = report[filename]

        if errors:
            # report each cppcheck error as a failing testcase
            for error in errors:
                data = {
                    'quoted_name': quoteattr(
                        '%s: %s (%s:%d:%d)'
                        % (
                            error['severity'],
                            error['id'],
                            filename,
                            error['line'],
                            error['col'],
                        )
                    ),
                    'testname': testname,
                    'quoted_message': quoteattr(error['msg']),
                }
                xml += (
                    """  <testcase
    name=%(quoted_name)s
    classname='%(testname)s'
  >
      <failure message=%(quoted_message)s/>
  </testcase>
"""
                    % data
                )

        else:
            # if there are no cpplint errors report a single successful test
            data = {
                'quoted_location': quoteattr(filename),
                'testname': testname,
            }
            xml += (
                """  <testcase
    name=%(quoted_location)s
    classname='%(testname)s'/>
"""
                % data
            )

    data = {
        'escaped_files': escape(''.join(['\n* %s' % r for r in sorted(report.keys())])),
    }
    xml += (
        """  <system-out>Checked files:%(escaped_files)s</system-out>
"""
        % data
    )

    xml += '</testsuite>\n'
    return xml


if __name__ == '__main__':
    sys.exit(main())
