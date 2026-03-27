# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import pathlib
import tempfile
import typing

from _pytest.fixtures import FixtureRequest
from _pytest.monkeypatch import MonkeyPatch
import ament_pep257.main as m
import pytest


if not m.ruff_installed or not m.pydocstyle_installed:
    pytest.fail('Neither ruff or pydocstyle installed')


class MainFunc(typing.Protocol):

    def __call__(self, argv: list[str] = ...) -> typing.Literal[0, 1]: ...


@pytest.fixture(params=['ruff', 'pydocstyle'])
def backend(request: FixtureRequest, monkeypatch: MonkeyPatch) -> MainFunc:
    if request.param == 'ruff':
        if not m.ruff_installed:
            pytest.skip('ruff not installed')
        monkeypatch.setattr(m, 'ruff_installed', True)
        monkeypatch.setattr(m, 'pydocstyle_installed', False)

    elif request.param == 'pydocstyle':
        if not m.pydocstyle_installed:
            pytest.skip('pydocstyle not installed')
        monkeypatch.setattr(m, 'ruff_installed', False)
        monkeypatch.setattr(m, 'pydocstyle_installed', True)

    return m.main


def test_invalid_file(backend: MainFunc) -> None:
    report = backend(['non_existent_file.py'])
    assert report == 1


def test_valid_file(backend: MainFunc) -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir = pathlib.Path(temp_dir)
        py_file = temp_dir / 'foobar.py'
        py_file.write_text('a = 1+2\n')

        report = backend([str(py_file)])
        assert report == 0


def test_valid_and_invalid_file(backend: MainFunc) -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir = pathlib.Path(temp_dir)
        py_file = temp_dir / 'foobar.py'
        py_file2 = temp_dir / 'barfoo.py'
        py_file.write_text('a = 1+2\n')

        report = backend([str(py_file), str(py_file2)])
        assert report == 1


def test_valid_with_violations(backend: MainFunc) -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir = pathlib.Path(temp_dir)
        py_file = temp_dir / 'foobar.py'

        py_file.write_text(
            'def foo():\n'
            '    """bad docstring"""\n'
            '    pass\n'
        )

        report = backend([str(py_file)])
        assert report == 1


def test_ignore_codes(backend: MainFunc) -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir = pathlib.Path(temp_dir)
        py_file = temp_dir / 'foobar.py'

        # This normally triggers D100 (missing docstring in module)
        py_file.write_text('')

        report = backend([str(py_file), '--ignore', 'D100'])
        assert report == 0


def test_directory_with_multiple_files(backend: MainFunc) -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir = pathlib.Path(temp_dir)
        files = {
            'good.py': 'a = 1\n',
            'bad.py': 'def f():\n    """foo"""\n    pass\n',
        }
        for name, content in files.items():
            (temp_dir / name).write_text(content)

        report = backend([str(temp_dir)])
        assert report == 1
