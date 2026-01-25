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
from typing import Final
import xml.etree.ElementTree as ET


from ament_index_python.packages import get_package_share_directory
import pytest
from pytest import Config
from pytest import Metafunc


DEPEND_TAGS: Final = (
    'depend',
    'build_depend',
    'test_depend',
    'exec_depend',
)

PACKAGE_XML: Final = 'package.xml'


def should_expand_package(name: str) -> bool:
    return name.startswith('ament_lint')


def get_package_xml_for_package(pkg_name: str) -> Path:
    return Path(get_package_share_directory(pkg_name)) / PACKAGE_XML


def get_depends_recursive(
    pkg_xml: Path,
    seen: set[str] | None = None,
) -> set[str]:
    seen = seen or set()

    tree = ET.parse(pkg_xml)
    root = tree.getroot()

    deps: set[str] = set()

    for tag in DEPEND_TAGS:
        for dep in root.findall(tag):
            if not dep.text:
                continue

            name = dep.text.strip()
            if name in seen:
                continue

            seen.add(name)
            deps.add(name)

            if should_expand_package(name):
                dep_xml = get_package_xml_for_package(name)
                deps |= get_depends_recursive(dep_xml, seen)

    return deps


def find_package_xml(start: Path) -> Path | None:
    for parent in [start, *start.parents]:
        pkg_xml = parent / PACKAGE_XML
        if pkg_xml.is_file():
            return pkg_xml
    return None


def pytest_configure(config: Config) -> None:
    config.addinivalue_line('markers',
                            'ament_lint_auto_py: marks tests as running all linter checks')


def pytest_generate_tests(metafunc: Metafunc) -> None:
    if 'ament_lint_ep' in metafunc.fixturenames:
        AMENT_LINT_AUTO_EXCLUDE = os.environ.get('AMENT_LINT_AUTO_EXCLUDE', '')
        EXCLUDED_LINTERS = {name.strip() for name in AMENT_LINT_AUTO_EXCLUDE.split(';') if name}

        test_file = Path(metafunc.definition.path)
        pkg_xml = find_package_xml(test_file)

        if pkg_xml is None:
            print('No package.xml found. Is this a ROS package?')
            return

        effective_depends = get_depends_recursive(pkg_xml)
        linters = entry_points(group='ament_lint')

        filtered_linters: list[EntryPoint] = []
        ids: list[int] = []

        for ep in linters:
            runner = ep.load()
            name = runner.NAME
            file_types = runner.FILE_TYPES

            if name not in effective_depends:
                print(f'Skipping {name} because it was not found in the package.xml')
                continue

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
    runner = ament_lint_ep.load()

    AMENT_LINT_AUTO_FILE_EXCLUDE = os.environ.get('AMENT_LINT_AUTO_FILE_EXCLUDE', '')
    EXCLUDED_FILE_GLOBS = {name.strip() for
                           name in AMENT_LINT_AUTO_FILE_EXCLUDE.split(';') if name}
    if EXCLUDED_FILE_GLOBS:
        args = ['--exclude']
        sys.argv.extend(EXCLUDED_FILE_GLOBS)
        return runner(args)

    return runner([])
