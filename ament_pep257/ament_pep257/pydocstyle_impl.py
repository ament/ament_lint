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

import logging
import sys

from ament_pep257.main import _filename_in_excludes
from pydocstyle import check

try:  # as of version 1.1.0
    from pydocstyle.config import ConfigurationParser
    from pydocstyle.violations import Error
    from pydocstyle.utils import log
except ImportError:  # try version 1.0.0
    from pydocstyle import ConfigurationParser
    from pydocstyle import Error
    from pydocstyle import log


log.setLevel(logging.INFO)


def generate_pep257_report(paths, excludes, ignore, select, convention, add_ignore, add_select):
    conf = ConfigurationParser()
    sys_argv = sys.argv
    sys.argv = [
        'main',
        '--match', r'.*\.py',
        '--match-dir', r'[^\._].*',
    ]
    if ignore:
        sys.argv += ['--ignore', ignore]
    elif select:
        sys.argv += ['--select', select]
    else:
        sys.argv += ['--convention', convention]
    if add_ignore:
        sys.argv += ['--add-ignore', add_ignore]
    if add_select:
        sys.argv += ['--add-select', add_select]
    sys.argv += paths
    conf.parse()
    sys.argv = sys_argv
    files_to_check = conf.get_files_to_check()

    report = []

    files_dict = {}
    # Unpack 3 values for pydocstyle <= 6.1.1 and 4 values for pydocstyle >= 6.2.0
    for filename, checked_codes, ignore_decorators, *_ in files_to_check:
        if _filename_in_excludes(filename, excludes):
            continue
        files_dict[filename] = {
            'select': checked_codes,
            'ignore_decorators': ignore_decorators,
        }

    for filename in sorted(files_dict.keys()):
        print('checking', filename)
        errors = []
        pep257_errors = check(
            [filename],
            **files_dict[filename])
        for pep257_error in pep257_errors:
            if isinstance(pep257_error, Error):
                errors.append({
                    'category': pep257_error.code,
                    'linenumber': pep257_error.line,
                    'message': pep257_error.message,
                })
                print(
                    '%s:%d %s: %s' %
                    (pep257_error.filename, pep257_error.line, pep257_error.definition,
                     pep257_error.message), file=sys.stderr)
            elif isinstance(pep257_error, SyntaxError):
                errors.append({
                    'category': str(type(pep257_error)),
                    'linenumber': '-',
                    'message': 'invalid syntax in file',
                })
                print('%s: invalid syntax' % filename, file=sys.stderr)
            else:
                errors.append({
                    'category': 'unknown',
                    'linenumber': '-',
                    'message': str(pep257_error),
                })
                print('%s: %s' % (filename, pep257_error), file=sys.stderr)
        report.append((filename, errors))
    return report
