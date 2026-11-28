# Copyright 2025 Open Source Robotics Foundation, Inc.
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

import json
from pathlib import Path

from ament_clang_tidy.main import get_source_dir_from_cmake_cache
from ament_clang_tidy.main import is_in_directory
from ament_clang_tidy.main import is_unittest_source
from ament_clang_tidy.main import main
from ament_lint.test_helpers import TempFileWriter


def test_is_unittest_source():
    package = 'ament_clang_tidy'
    assert is_unittest_source(
        package, r'C:\workspace\src\ament_clang_tidy\test\test_execution.cpp')
    assert is_unittest_source(
        package, '/workspace/src/ament_clang_tidy/test/test_execution.cpp')
    assert not is_unittest_source(
        package, r'C:\workspace\src\ament_clang_tidy\src\main.cpp')


def test_is_in_directory(tmp_path):
    src = tmp_path / 'src' / 'pkg'
    assert is_in_directory(str(src / 'src' / 'main.cpp'), str(src))
    assert not is_in_directory(str(tmp_path / 'build' / 'pkg' / 'gen.cpp'), str(src))
    # a sibling with a common prefix is not inside the directory
    assert not is_in_directory(str(tmp_path / 'src' / 'pkg_extra' / 'a.cpp'), str(src))


def test_get_source_dir_from_cmake_cache(tmp_path):
    assert get_source_dir_from_cmake_cache(str(tmp_path)) is None
    (tmp_path / 'CMakeCache.txt').write_text(
        '# comment\n'
        'CMAKE_BUILD_TYPE:STRING=Release\n'
        'CMAKE_HOME_DIRECTORY:INTERNAL=/ws/src/pkg\n')
    assert get_source_dir_from_cmake_cache(str(tmp_path)) == '/ws/src/pkg'


def test_generated_files_outside_source_dir_are_skipped(tmp_path):
    """Files generated into the build directory must not be checked."""
    src_dir = tmp_path / 'src' / 'pkg'
    build_dir = tmp_path / 'build' / 'pkg'
    src_dir.mkdir(parents=True)
    (build_dir / 'rclcpp_components').mkdir(parents=True)
    (build_dir / 'CMakeCache.txt').write_text(
        'CMAKE_HOME_DIRECTORY:INTERNAL=%s\n' % src_dir)

    src_file = src_dir / 'main.cpp'
    src_file.write_text('int main() { return 0; }\n')
    generated_file = build_dir / 'rclcpp_components' / 'node_main_foo.cpp'
    generated_file.write_text(
        'namespace a {}\nusing namespace a;\nint main() { return 0; }\n')

    (build_dir / 'compile_commands.json').write_text(json.dumps([
        {'directory': str(build_dir), 'command': 'c++ -c %s' % f, 'file': str(f)}
        for f in (src_file, generated_file)
    ]))

    xunit_file = tmp_path / 'clang_tidy.xunit.xml'
    rc = main(argv=[
        str(build_dir), '--xunit-file', str(xunit_file),
        '--config', str(_write_config(tmp_path, 'google-build-using-namespace')),
    ])
    assert rc == 0, 'Generated file outside the source directory was checked'
    xunit = xunit_file.read_text()
    assert 'main.cpp' in xunit
    assert 'node_main_foo.cpp' not in xunit


def _write_config(directory, checks):
    config = directory / '.clang-tidy'
    config.write_text("Checks: '-*,%s'\nWarningsAsErrors: ''\n" % checks)
    return config


def test_clang_tidy_execution():
    """Test that clang tidy can be executed on a simple C++ file, via ament_clang_tidy."""
    with TempFileWriter('int main() { return 0; }', 'test.cpp') as temp_file_path:
        temp_dir = Path(temp_file_path).parent

        # Create compile_commands.json
        compile_commands = [
            {
                'directory': str(temp_dir),
                'command': 'c++ -c test.cpp',
                'file': str(temp_file_path),
            }
        ]
        compile_commands_json = json.dumps(compile_commands)
        with TempFileWriter(
            compile_commands_json, 'compile_commands.json'
        ) as compile_commands_path:
            rc = main(argv=['ament_clang_tidy', str(compile_commands_path)])
            assert rc == 0, 'Clang tidy found issues'
