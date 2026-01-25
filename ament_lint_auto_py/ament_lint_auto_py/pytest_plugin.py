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
from pathlib import Path
import sys

import pytest
from pytest import Config
from pytest import Metafunc


def pytest_configure(config: Config) -> None:
    config.addinivalue_line('markers',
                            'ament_lint_auto_py: marks tests as running all linter checks')


def pytest_generate_tests(metafunc: Metafunc) -> None:
    if 'ament_lint_ep' in metafunc.fixturenames:
        linters = entry_points(group='ament_lint')

        filtered_linters: list[EntryPoint] = []
        ids: list[int] = []

        for ep in linters:
            func = ep.load()
            name = func.NAME
            file_types = func.FILE_TYPES

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
    sys.argv = [func.NAME]
    return func()
