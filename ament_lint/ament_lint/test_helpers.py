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

import tempfile
import os

class TempFileWriter:
    """Context manager that writes a string to a temp file and cleans up on exit."""
    
    def __init__(self, content, filename):
        self.content = content
        self.filename = filename
        self.temp_dir = None
        self.temp_file = None
    
    def __enter__(self):
        # Create a new temporary directory
        self.temp_dir = tempfile.mkdtemp()
        # Create the temp file path
        self.temp_file = os.path.join(self.temp_dir, self.filename)
        # Write content to the file
        with open(self.temp_file, 'w') as f:
            f.write(self.content)
        return self.temp_file
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        # Remove the file
        if self.temp_file and os.path.exists(self.temp_file):
            os.remove(self.temp_file)
        # Remove the directory
        if self.temp_dir and os.path.exists(self.temp_dir):
            os.rmdir(self.temp_dir)
        return False
