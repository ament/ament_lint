# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_copyright.main import main


cases_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'cases')


def test_apache2_standard():
    rc = main(argv=[os.path.join(cases_path, 'apache2_license')])
    assert rc == 0, 'Found errors'


def test_boost1_cpp():
    rc = main(argv=[os.path.join(cases_path, 'boost1/case2.cpp')])
    assert rc == 0, 'Found errors'


def test_boost1_py():
    rc = main(argv=[os.path.join(cases_path, 'boost1/case.py')])
    assert rc == 0, 'Found errors'


def test_bsd_standard():
    rc = main(argv=[os.path.join(cases_path, 'bsd_license')])
    assert rc == 0, 'Found errors'


def test_bsd_indented():
    rc = main(argv=[os.path.join(cases_path, 'bsd_license_indented')])
    assert rc == 0, 'Found errors'


def test_bsd_tabs():
    rc = main(argv=[os.path.join(cases_path, 'bsd_license_tabs')])
    assert rc == 0, 'Found errors'


def test_3bsd_cpp():
    rc = main(argv=[os.path.join(cases_path, '3clause_bsd/case2.cpp')])
    assert rc == 0, 'Found errors'


def test_3bsd_py():
    rc = main(argv=[os.path.join(cases_path, '3clause_bsd/case.py')])
    assert rc == 0, 'Found errors'
