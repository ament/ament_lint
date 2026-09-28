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

from xml.etree import ElementTree

from ament_clang_tidy.main import get_xunit_content
from ament_clang_tidy.main import parse_diagnostics


EXTENSIONS = ['c', 'cc', 'cpp', 'cxx', 'h', 'hh', 'hpp', 'hxx']


def test_parse_diagnostics_with_unix_path():
    output = (
        '/workspace/src/example.cpp:12:3: warning: use nullptr [modernize-use-nullptr]\n'
        '  return NULL;\n'
        '         ^~~~\n'
    )

    report = parse_diagnostics(output, EXTENSIONS)

    assert list(report) == ['/workspace/src/example.cpp']
    assert report['/workspace/src/example.cpp'] == [{
        'line_no': '12',
        'offset_in_line': '3',
        'error_msg': 'use nullptr [modernize-use-nullptr]',
        'code_correct_rec': '  return NULL;\n         ^~~~\n',
    }]


def test_windows_diagnostic_is_preserved_in_xunit():
    path = r'C:\pixi_ws\ros2-windows\include\rclcpp\rclcpp/any_subscription_callback.hpp'
    message = (
        "'set_deprecated<sensor_msgs::msg::Image_>' is deprecated: "
        "use 'void(std::shared_ptr<const MessageT>)' instead "
        '[clang-diagnostic-deprecated-declarations]')
    output = (
        f'{path}:441:7: warning: {message}\n'
        '  set_deprecated(callback);\n'
        '  ^\n'
        f'{path}:123:4: note: set_deprecated has been marked deprecated here\n'
    )

    report = parse_diagnostics(output, EXTENSIONS)
    xml = get_xunit_content(report, 'clang_tidy', 0.1)
    testsuite = ElementTree.fromstring(xml)
    testcase = testsuite.find('testcase')
    failure = testcase.find('failure')

    assert list(report) == [path]
    assert report[path][0]['error_msg'] == message
    assert testcase.attrib['name'] == f'{path}:441:7'
    assert failure.attrib['message'] == message
    assert '  set_deprecated(callback);' in failure.text
    assert f'{path}:123:4: note:' in failure.text
