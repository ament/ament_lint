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

import pytest


# TODO: AMENT_LINT_AUTO_EXCLUDE suppor
@pytest.mark.ament_lint_auto_py
@pytest.mark.linter
def test_ament_lint_auto(run_entry_point) -> None:
    """Run all installed linter dynamically."""
    rc = run_entry_point()
    assert rc == 0, f'Linter[{run_entry_point.NAME}] failed with exit code {rc}'
