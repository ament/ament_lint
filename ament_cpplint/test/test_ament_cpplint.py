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

from ament_cpplint import cpplint
from ament_cpplint.main import main


def test_cpplint_version():
    assert cpplint.__VERSION__ == '2.0.2'


def test_cpp20_headers(tmp_path):
    source = tmp_path / 'cpp20.cpp'
    source.write_text(
        '// Copyright 2026 Open Source Robotics Foundation, Inc.\n'
        '\n'
        '#include <array>\n'
        '#include <bit>\n'
        '#include <cstdint>\n'
        '#include <cstring>\n'
        '#include <optional>\n'
        '#include <ranges>\n'
        '#include <span>\n'
        '#include <stdexcept>\n'
        '#include <string>\n'
        '#include <string_view>\n'
        '#include <utility>\n'
        '\n'
        'int main()\n'
        '{\n'
        '  return 0;\n'
        '}\n'
    )

    assert main(['--quiet', str(source)]) == 0


def test_lint_error_is_reported(tmp_path):
    source = tmp_path / 'bad_style.cpp'
    source.write_text('int\tmain() { return 0; }\n')

    assert main(['--quiet', str(source)]) == 1
