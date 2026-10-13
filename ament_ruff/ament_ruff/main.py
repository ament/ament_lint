#!/usr/bin/env python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import sys

import subprocess

def main(argv=sys.argv[1:]):
    result = subprocess.run(
        [
            "ruff",
            "format",
            "--config",
            "/opt/overlay_ws/src/navigation2/ament_lint/ament_ruff/ament_ruff/ament_ruff.toml",
        ],
        capture_output=True,
        text=True,
    )
    lint_result = subprocess.run(
        [
            "ruff",
            "check",
            "--config",
            "/opt/overlay_ws/src/navigation2/ament_lint/ament_ruff/ament_ruff/ament_ruff.toml",
        ],
        capture_output=True,
        text=True,
    )
    sys.stdout.write(result.stdout)
    sys.stdout.write(lint_result.stdout)


if __name__ == "__main__":
    sys.exit(main())
