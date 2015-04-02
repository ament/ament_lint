# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr


def get_xunit_content(report, testname, elapsed):
    test_count = sum([max(len(r), 1) for r in report.values()])
    error_count = sum([len(r) for r in report.values()])
    data = {
        'testname': testname,
        'test_count': test_count,
        'error_count': error_count,
        'time': '%.3f' % round(elapsed, 3),
    }
    xml = '''<?xml version="1.0" encoding="UTF-8"?>
<testsuite
  name="%(testname)s"
  tests="%(test_count)d"
  failures="%(error_count)d"
  time="%(time)s"
>
''' % data

    for filename in sorted(report.keys()):
        replacements = report[filename]

        if replacements:
            # report each replacement as a failing testcase
            for replacement in replacements:
                data = {
                    'quoted_location': quoteattr(
                        '%s:%d:%d' % (
                            filename, replacement['line_no'],
                            replacement['offset_in_line'])),
                    'testname': testname,
                    'quoted_message': quoteattr(
                        'Replace [%s] with [%s]' %
                        (replacement['original'], replacement['replacement'])
                    ),
                    'cdata': '\n'.join([
                        '%s:%d:%d' % (
                            filename, replacement['line_no'],
                            replacement['offset_in_line']),
                        replacement['deletion'],
                        replacement['addition'],
                    ]),
                }
                xml += '''  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s><![CDATA[%(cdata)s]]></failure>
  </testcase>
''' % data

        else:
            # if there are no replacements report a single successful test
            data = {
                'quoted_location': quoteattr(filename),
                'testname': testname,
            }
            xml += '''  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"
    status="No errors"/>
''' % data

    # output list of checked files
    data = {
        'escaped_files': escape(''.join(['\n* %s' % r
                                         for r in sorted(report.keys())])),
    }
    xml += '''  <system-out>Checked files:%(escaped_files)s</system-out>
''' % data

    xml += '</testsuite>'
    return xml
