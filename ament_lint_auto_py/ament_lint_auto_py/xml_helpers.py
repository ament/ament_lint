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

from pathlib import Path
from typing import Final
import xml.etree.ElementTree as ET


from ament_index_python.packages import get_package_share_directory


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


def package_has_ament_lint_auto_py(pkg_xml: Path) -> bool:
    tree = ET.parse(pkg_xml)
    root = tree.getroot()
    for tag in DEPEND_TAGS:
        for dep in root.findall(tag):
            if dep.text and dep.text.strip() == 'ament_lint_auto_py':
                return True
    return False
