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

from ament_clang_format.main import main


EMPTY_CONTENTS = ''
PROPER_CONTENTS = '#include <this/file/should/be/unchanged>\n'
MALFORMED_CONTENTS_BEFORE = 'void this_should_get_formatted(     );\n'
MALFORMED_CONTENTS_AFTER = 'void this_should_get_formatted();\n'


def test_empty_files(tmp_path, capsys):
    """Check that formatting suggestions are correct in the presence of an empty file."""
    empty_path = tmp_path / 'empty.cpp'
    proper_path = tmp_path / 'proper.cpp'
    malformed_path = tmp_path / 'malformed.cpp'

    empty_path.write_text(EMPTY_CONTENTS, encoding='utf_8')
    proper_path.write_text(PROPER_CONTENTS, encoding='utf_8')
    malformed_path.write_text(MALFORMED_CONTENTS_BEFORE, encoding='utf_8')

    main([str(empty_path), str(proper_path), str(malformed_path)])

    captured = capsys.readouterr()
    assert MALFORMED_CONTENTS_AFTER in captured.err


def test_empty_files_reformat(tmp_path):
    """Check that reformatting applies correctly in the presence of an empty file."""
    empty_path = tmp_path / 'empty.cpp'
    proper_path = tmp_path / 'proper.cpp'
    malformed_path = tmp_path / 'malformed.cpp'

    empty_path.write_text(EMPTY_CONTENTS, encoding='utf_8')
    proper_path.write_text(PROPER_CONTENTS, encoding='utf_8')
    malformed_path.write_text(MALFORMED_CONTENTS_BEFORE, encoding='utf_8')

    main(['--reformat', str(empty_path), str(proper_path), str(malformed_path)])

    assert empty_path.read_text(encoding='utf_8') == EMPTY_CONTENTS
    assert proper_path.read_text(encoding='utf_8') == PROPER_CONTENTS
    assert malformed_path.read_text(encoding='utf_8') == MALFORMED_CONTENTS_AFTER
