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

import io
import shutil
import urllib.request

from ament_xmllint.main import get_local_schema_path
from ament_xmllint.main import main
import pytest


VALID_XML = '<root><child/></root>\n'
INVALID_XML = '<root><child></root>\n'
SCHEMA_URL = 'http://example.invalid/schema.xsd'

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


def test_schema_is_downloaded_with_a_timeout(tmp_path, monkeypatch):
    """Check that a remote schema is fetched with a timeout and stored locally."""
    timeouts = []

    def urlopen(url, timeout=None):
        timeouts.append(timeout)
        return io.BytesIO(b'<schema/>')

    monkeypatch.setattr(urllib.request, 'urlopen', urlopen)

    local_path = get_local_schema_path(SCHEMA_URL, str(tmp_path), set())

    assert local_path != SCHEMA_URL
    with open(local_path, 'rb') as f:
        assert f.read() == b'<schema/>'
    assert len(timeouts) == 1
    assert timeouts[0] is not None


def test_failed_schema_download_is_not_retried(tmp_path, monkeypatch, capsys):
    """Check that a failed download falls back to the URL and is only attempted once."""
    calls = []

    def urlopen(url, timeout=None):
        calls.append(url)
        raise TimeoutError('timed out')

    monkeypatch.setattr(urllib.request, 'urlopen', urlopen)
    failed_urls = set()

    assert get_local_schema_path(SCHEMA_URL, str(tmp_path), failed_urls) == SCHEMA_URL
    assert get_local_schema_path(SCHEMA_URL, str(tmp_path), failed_urls) == SCHEMA_URL

    assert calls == [SCHEMA_URL]
    assert 'failed to download schema' in capsys.readouterr().err


def test_interrupted_schema_download_is_not_cached(tmp_path, monkeypatch):
    """Check that a download failing while reading the response leaves no file behind."""

    class StalledResponse(io.BytesIO):

        def read(self, *args):
            raise TimeoutError('timed out')

    monkeypatch.setattr(
        urllib.request, 'urlopen', lambda url, timeout=None: StalledResponse())

    assert get_local_schema_path(SCHEMA_URL, str(tmp_path), set()) == SCHEMA_URL
    assert list(tmp_path.iterdir()) == []
