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

import importlib
import json
import subprocess
import xml.etree.ElementTree as ET

import pytest

tidy = importlib.import_module('ament_clang_tidy.main')


@pytest.mark.parametrize('jobs', [1, 2])
@pytest.mark.parametrize('output', [
    b'', b'clang: error: invalid option\n',
    b'C:\\src\\bad.cpp:1:1: error: failed\n',
])
def test_failed_invocation(tmp_path, monkeypatch, jobs, output):
    """A failed command must fail both the runner and its xUnit report."""
    database = tmp_path / 'compile_commands.json'
    good = str(tmp_path / 'good.cpp')
    bad = str(tmp_path / 'bad.cpp')
    database.write_text(json.dumps([{'file': good}, {'file': bad}]))
    report = tmp_path / 'report.xml'
    monkeypatch.setattr(tidy, 'find_executable', lambda names: 'clang-tidy')

    def invoke(command, **kwargs):
        if command[-1] == bad:
            raise subprocess.CalledProcessError(1, command, output=output)
        return b''

    monkeypatch.setattr(tidy.subprocess, 'check_output', invoke)
    assert tidy.main([str(database), '--jobs', str(jobs),
                      '--xunit-file', str(report)]) == 1
    suite = ET.parse(report).getroot()
    assert int(suite.attrib['failures']) >= 1
    assert any('error code 1' in failure.attrib['message']
               for failure in suite.iter('failure'))
    assert any(case.attrib['name'] == good and case.find('failure') is None
               for case in suite.iter('testcase'))


def test_successful_invocation(tmp_path, monkeypatch):
    """An invocation with no diagnostics and a zero status still passes."""
    database = tmp_path / 'compile_commands.json'
    database.write_text(json.dumps([{'file': str(tmp_path / 'good.cpp')}]))
    report = tmp_path / 'report.xml'
    monkeypatch.setattr(tidy, 'find_executable', lambda names: 'clang-tidy')
    monkeypatch.setattr(tidy.subprocess, 'check_output', lambda *a, **kw: b'')
    assert tidy.main([str(database), '--xunit-file', str(report)]) == 0
    assert ET.parse(report).getroot().attrib['failures'] == '0'
