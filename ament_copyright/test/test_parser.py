# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_copyright.parser import search_copyright_information


def test_search_copyright_information_incorrect():
    """Test searching for copyright information with a typo in the copyright information."""
    copyrights, remaining_block = search_copyright_information(
        'CopyrightTypo 2020 Open Source Robotics Foundation, Inc.'
    )
    assert len(copyrights) == 0


def test_search_copyright_information_capitalization1():
    """
    Test searching for copyright information with capitalized copyright information.

    Word 'copyright': capitalized
    Abbreviation '(c)': present and lowercase
    """
    copyrights, remaining_block = search_copyright_information(
        'Copyright 2020 Open Source Robotics Foundation, Inc.')
    assert len(copyrights) > 0


def test_search_copyright_information_capitalization2():
    """
    Test searching for copyright information with capitalized copyright information.

    Word 'copyright': capitalized
    Abbrebiation '(c)': present and lowercase
    """
    copyrights, remaining_block = search_copyright_information(
        'Copyright (c) 2020 Open Source Robotics Foundation, Inc.')
    assert len(copyrights) > 0


def test_search_copyright_information_capitalization3():
    """
    Test searching for copyright information with capitalized copyright information.

    Word 'copyright': capitalized
    Abbrebiation '(c)': present and uppercase
    """
    copyrights, remaining_block = search_copyright_information(
        'Copyright (C) 2020 Open Source Robotics Foundation, Inc.')
    assert len(copyrights) > 0


def test_search_copyright_information_lowercase1():
    """
    Test searching for copyright information with lowercase copyright information.

    Word 'copyright': lowercase
    Abbrebiation '(c)': absent
    """
    copyrights, remaining_block = search_copyright_information(
        'copyright 2020 Open Source Robotics Foundation, Inc.')
    assert len(copyrights) > 0


def test_search_copyright_information_lowercase2():
    """
    Test searching for copyright information with lowercase copyright information and abbreviation.

    Word 'copyright': lowercase
    Abbrebiation '(c)': lowercase
    """
    copyrights, remaining_block = search_copyright_information(
        'copyright (c) 2020 Open Source Robotics Foundation, Inc.')
    assert len(copyrights) > 0
