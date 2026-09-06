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

from unittest.mock import call
from unittest.mock import patch

from ament_clang_tidy.main import find_executable


def test_find_executable_uses_platform_lookup():
    executable = r'C:\tools\clang-tidy.exe'
    with patch(
        'ament_clang_tidy.main.shutil.which',
        side_effect=[executable],
    ) as which:
        assert find_executable(['clang-tidy']) == executable

    assert which.call_args_list == [call('clang-tidy')]
