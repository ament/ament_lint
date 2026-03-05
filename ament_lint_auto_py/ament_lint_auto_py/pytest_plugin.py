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

from importlib.metadata import entry_points
from importlib.metadata import EntryPoint
from pathlib import Path
from typing import Literal

from ament_lint_auto_py.xml_helpers import find_package_xml
from ament_lint_auto_py.xml_helpers import get_depends_recursive
from ament_lint_auto_py.xml_helpers import package_has_ament_lint_auto_py
from pytest import Config
from pytest import Item
from pytest import Parser
from pytest import Session


def pytest_configure(config: Config) -> None:
    config.addinivalue_line(
        'markers', 'ament_lint_auto_py: marks tests as running all linter checks'
    )


class AmentLintItem(Item):
    def __init__(
        self,
        *,
        entry_point: EntryPoint,
        file_excludes: list[str],
        **kwargs,
    ) -> None:
        super().__init__(**kwargs)
        self.entry_point = entry_point
        self.file_excludes = file_excludes

    def runtest(self) -> None:
        runner = self.entry_point.load()

        args: list[str] = []
        if self.file_excludes:
            args.append('--exclude')
            args.extend(self.file_excludes)

        rc = runner(args)()
        if rc != 0:
            raise AssertionError(
                f'Linter[{runner.NAME}] failed with exit code {rc}'
            )

    def reportinfo(self) -> tuple[Path, Literal[0], str]:
        return self.path, 0, f'ament_lint: {self.name}'


def pytest_collection_modifyitems(session: Session, config: Config, items: list[Item]) -> None:
    excluded = set(config.getini('ament_lint_auto_exclude'))
    file_excludes = config.getini('ament_lint_auto_file_exclude')

    pkg_xml = find_package_xml(config.rootpath)
    if pkg_xml is None:
        return  # Not in a ROS package

    if not package_has_ament_lint_auto_py(pkg_xml):
        return  # Package does not opt-in

    effective_depends = get_depends_recursive(pkg_xml)
    linters = entry_points(group='ament_lint')

    for ep in linters:
        runner = ep.load()

        if runner.NAME not in effective_depends:
            continue  # skipping linter if not declared in depends of a package.xml

        if runner.NAME in excluded:
            continue  # skipping linter if declared in ament_lint_auto_exclude

        # skip linters if no matching files exist
        found_file = False
        for pattern in runner.FILE_TYPES:
            if any(config.rootpath.rglob(pattern)):
                found_file = True
                break
        if not found_file:
            continue

        items.append(
            AmentLintItem.from_parent(
                parent=session,
                name=runner.NAME,
                entry_point=ep,
                file_excludes=file_excludes,
                path=config.rootpath,
            )
        )


def pytest_addoption(parser: Parser):
    parser.addini(
        'ament_lint_auto_exclude',
        'Linters to exclude from ament_lint_auto_py',
        type='linelist',
        default=[],
    )
    parser.addini(
        'ament_lint_auto_file_exclude',
        'File globs to exclude from ament linters',
        type='linelist',
        default=[],
    )
