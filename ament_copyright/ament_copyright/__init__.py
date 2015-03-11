# Copyright 2014 Open Source Robotics Foundation, Inc.
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


def check_file_for_copyright(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        comment = ''
        block_comment = False
        for line in lines:
            line = line.strip()
            # skip empty lines
            if not line:
                continue
            # identify block comments
            if line.startswith('/*'):
                block_comment = True
            # collect all comment lines
            if block_comment or line.startswith('#') or line.startswith('//'):
                comment += '%s\n' % line
                if block_comment and '*/' in line:
                    break
            else:
                break
        return 'copyright' in comment.lower()


def get_xunit_content(report, testname, elapsed):
    test_count = len(report)
    error_count = len([r for r in report if not r[1]])
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

    for (filename, has_copyright) in report:

        data = {
            'quoted_filename': quoteattr(filename),
            'testname': testname,
        }
        if not has_copyright:
            # report missing copyright as a failing testcase
            xml += '''  <testcase
    name=%(quoted_filename)s
    classname="%(testname)s"
  >
      <failure message="could not find copyright reference"/>
  </testcase>
''' % data

        else:
            # if there is a copyright report a single successful test
            xml += '''  <testcase
    name=%(quoted_filename)s
    classname="%(testname)s"
    status="No errors"/>
''' % data

    # output list of checked files
    data = {
        'escaped_files': escape(''.join(['\n* %s' % r[0] for r in report])),
    }
    xml += '''  <system-out>Checked files:%(escaped_files)s</system-out>
''' % data

    xml += '</testsuite>'
    return xml
