#!/usr/bin/env python3

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

from __future__ import print_function

import argparse
import os
import pep8
import sys
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr


def main(argv=sys.argv[1:]):
    config_file = os.path.join(
        os.path.dirname(__file__), 'configuration', 'ament_pep8.ini')

    parser = argparse.ArgumentParser(
        description='Check code against the style conventions in PEP 8.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--config',
        metavar='path',
        default=config_file,
        dest='config_file',
        help='The config file')
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help="The files or directories to check. For directories files ending "
             "in '.py' will be considered.")
    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    args = parser.parse_args(argv)

    if not os.path.exists(args.config_file):
        print("Could not config file '%s'" % args.config_file, file=sys.stderr)
        return 1

    report = generate_pep8_report(args.config_file, args.paths)

    # print statistics about errors
    if report.total_errors:
        print('')
        report.print_statistics()

    # print summary
    print('')
    if not report.total_errors:
        print('No errors or warnings')
        rc = 0
    else:
        errors = report.get_count('E')
        warnings = report.get_count('W')
        print('%d errors, %d warnings' % (errors, warnings))
        rc = 1

    # generate xunit file
    if args.xunit_file:
        folder_name = os.path.basename(os.path.dirname(args.xunit_file))
        file_name = os.path.basename(args.xunit_file)
        suffix = '.xml'
        if file_name.endswith(suffix):
            file_name = file_name[0:-len(suffix)]
        testname = '%s.%s' % (folder_name, file_name)

        xml = get_xunit_content(report, testname)
        path = os.path.dirname(os.path.abspath(args.xunit_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.xunit_file, 'w') as f:
            f.write(xml)

    return rc


def generate_pep8_report(config_file, paths):
    pep8style = CustomStyleGuide(
        repeat=True,
        show_source=True,
        verbose=True,
        reporter=CustomReport,
        config_file=config_file,
    )
    return pep8style.check_files(paths)


def get_xunit_content(report, testname):
    data = {
        'testname': testname,
        'test_count': max(report.total_errors, 1),
        'error_count': report.total_errors,
        'time': '%.3f' % round(report.elapsed, 3),
    }
    xml = '''<?xml version="1.0" encoding="UTF-8"?>
<testsuite
  name="%(testname)s"
  tests="%(test_count)d"
  failures="%(error_count)d"
  time="%(time)s"
>
''' % data

    if report.errors:
        # report each pep8 error/warning as a failing testcase
        for error in report.errors:
            data = {
                'quoted_name': quoteattr(
                    error['error_code'] +
                    ' (%(path)s:%(row)d:%(column)d)' % error),
                'testname': testname,
                'quoted_message': quoteattr(
                    '%(error_message)s:\n%(source_line)s' % error),
            }
            xml += '''  <testcase
    name=%(quoted_name)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s/>
  </testcase>
''' % data

    else:
        # if there are no pep8 errors/warnings report a single successful test
        data = {
            'testname': testname,
        }
        xml += '''  <testcase
    name="pep8"
    classname="%(testname)s"
    status="No errors or warnings"/>
''' % data

    # output list of checked files
    data = {
        'escaped_files': escape(''.join(['\n* %s' % f for f in report.files])),
    }
    xml += '''  <system-out>Checked files:%(escaped_files)s</system-out>
''' % data

    xml += '</testsuite>\n'
    return xml


class CustomStyleGuide(pep8.StyleGuide):

    def input_file(self, filename, **kwargs):
        self.options.reporter.files.append(filename)
        return super(CustomStyleGuide, self).input_file(filename, **kwargs)


class CustomReport(pep8.StandardReport):

    errors = []
    files = []

    def error(self, line_number, offset, text, check):
        code = super(CustomReport, self).error(
            line_number, offset, text, check)
        line = self.lines[line_number - 1] \
            if line_number <= len(self.lines) else ''
        self.errors.append({
            'path': self.filename,
            'row': self.line_offset + line_number,
            'column': offset + 1,
            'error_code': code,
            'error_message': text,
            'source_line': line.splitlines()[0],
        })
        return code


if __name__ == '__main__':
    sys.exit(main())