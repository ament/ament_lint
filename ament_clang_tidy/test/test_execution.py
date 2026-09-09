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

from ament_clang_tidy.main import main
from ament_lint.test_helpers import TempFileWriter


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
