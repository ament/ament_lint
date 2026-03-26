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


import json
import os
import subprocess
import sys

from ament_pep257.main import _filename_in_excludes


def generate_ruff_report(paths: list[str], excludes: list[str], ignore: str, select: str,
                         convention: str, add_ignore: str, add_select: str):

    # If files don't exist fail fast
    report = []
    existing_paths = []
    for path in paths:
        if not os.path.exists(path):
            report.append((path, [{
                'category': 'unknown',
                'linenumber': '-',
                'message': 'file does not exist'
            }]))
        else:
            existing_paths.append(path)

    if not existing_paths:
        return report

    cmd = ['ruff', 'check', '--output-format', 'json']

    if ignore:
        cmd += ['--ignore', ignore]
    elif select:
        cmd += ['--select', select]
    else:
        cmd += ['--convention', convention]

    if add_ignore:
        cmd += ['--ignore', add_ignore]

    if add_select:
        cmd += ['--select', add_select]

    # excludes
    for e in excludes:
        cmd += ['--exclude', e]

    # paths
    cmd += existing_paths

    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
    )

    data = json.loads(result.stdout)

    files_dict = {}

    for item in data:
        filename = os.path.relpath(item['filename'])

        if _filename_in_excludes(filename, excludes):
            continue

        files_dict.setdefault(filename, []).append(item)

    for filename in sorted(files_dict.keys()):
        print('checking', filename)
        errors = []

        for err in files_dict[filename]:
            errors.append({
                'category': err.get('code', 'unknown'),
                'linenumber': err.get('location', {}).get('row', '-'),
                'message': err.get('message', ''),
            })

            print(
                '%s:%s %s: %s' % (
                    filename,
                    err.get('location', {}).get('row', '-'),
                    err.get('code', 'unknown'),
                    err.get('message', ''),
                ),
                file=sys.stderr,
            )

        report.append((filename, errors))

    # If no violations count as a pass
    reported_files = {filename for filename, _ in report}

    for path in existing_paths:
        if os.path.isdir(path):
            for root, _, files in os.walk(path):
                for f in files:
                    if f.endswith('.py'):
                        rel = os.path.relpath(os.path.join(root, f))
                        if rel not in reported_files and not _filename_in_excludes(rel, excludes):
                            report.append((rel, []))
        else:
            rel = os.path.relpath(path)
            if rel not in reported_files and not _filename_in_excludes(rel, excludes):
                report.append((rel, []))

    return report
