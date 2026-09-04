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

"""Tests for the ament_xmllint entry point."""

import shutil

from ament_xmllint.main import main
import pytest


VALID_XML = '<root><child/></root>\n'
INVALID_XML = '<root><child></root>\n'

requires_xmllint = pytest.mark.skipif(
    shutil.which('xmllint') is None, reason="requires the 'xmllint' executable")


@requires_xmllint
def test_invalid_file_is_reported_when_checked_first(tmp_path, monkeypatch):
    """Check that an invalid file is reported even when it is not checked last."""
    (tmp_path / 'a_invalid.xml').write_text(INVALID_XML)
    (tmp_path / 'b_valid.xml').write_text(VALID_XML)
    monkeypatch.chdir(tmp_path)

    assert main(argv=[]) == 1


@requires_xmllint
def test_valid_files_pass(tmp_path, monkeypatch):
    """Check that a directory containing only valid files passes."""
    (tmp_path / 'a_valid.xml').write_text(VALID_XML)
    (tmp_path / 'b_valid.xml').write_text(VALID_XML)
    monkeypatch.chdir(tmp_path)

    assert main(argv=[]) == 0


def test_missing_xmllint_returns_error_code(tmp_path, monkeypatch):
    """Check that a missing xmllint executable yields an error code, not a string."""
    (tmp_path / 'a_valid.xml').write_text(VALID_XML)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(shutil, 'which', lambda name: None)

    assert main(argv=[]) == 1
