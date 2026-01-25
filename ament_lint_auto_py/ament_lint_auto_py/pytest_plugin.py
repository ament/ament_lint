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

from collections.abc import Callable
from importlib.metadata import entry_points
from importlib.metadata import EntryPoint
import os
from pathlib import Path
import sys

import pytest
from pytest import Config
from pytest import Metafunc


def pytest_configure(config: Config) -> None:
    config.addinivalue_line('markers',
                            'ament_lint_auto_py: marks tests as running all linter checks')


def pytest_generate_tests(metafunc: Metafunc) -> None:
    AMENT_LINT_AUTO_EXCLUDE = os.environ.get('AMENT_LINT_AUTO_EXCLUDE', '')
    EXCLUDED_LINTERS = {name.strip() for name in AMENT_LINT_AUTO_EXCLUDE.split(';') if name}

    if 'ament_lint_ep' in metafunc.fixturenames:
        linters = entry_points(group='ament_lint')

        filtered_linters: list[EntryPoint] = []
        ids: list[int] = []

        for ep in linters:
            func = ep.load()
            name = func.NAME
            file_types = func.FILE_TYPES

            if name in EXCLUDED_LINTERS:
                print(f'Skipping {name} because it is in '
                      f'AMENT_LINT_AUTO_EXCLUDE:={AMENT_LINT_AUTO_EXCLUDE}.')
                continue

            paths: list[Path] = []
            for ext in file_types:
                paths.extend(Path('.').rglob(f'{ext}'))

            if not paths:
                print(f'Skipping {name} because no files of type {file_types} exist.')
                continue

            filtered_linters.append(ep)
            ids.append(name)

        metafunc.parametrize('ament_lint_ep', filtered_linters, ids=ids)


@pytest.fixture
def run_entry_point(ament_lint_ep: EntryPoint) -> Callable[[], int]:
    func = ament_lint_ep.load()

    AMENT_LINT_AUTO_FILE_EXCLUDE = os.environ.get('AMENT_LINT_AUTO_FILE_EXCLUDE', '')
    EXCLUDED_FILE_GLOBS = {name.strip() for
                           name in AMENT_LINT_AUTO_FILE_EXCLUDE.split(';') if name}
    if EXCLUDED_FILE_GLOBS:
        args = ['--exclude']
        sys.argv.extend(EXCLUDED_FILE_GLOBS)
        return func(args)

    return func([])
