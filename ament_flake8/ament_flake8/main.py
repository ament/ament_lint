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

from __future__ import print_function

import argparse
import os
import sys
import time
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr

import flake8
from distutils.version import LooseVersion


def main(argv=sys.argv[1:]):
    if LooseVersion(flake8.__version__) < '3.0':
        print(
            "Found Flake8 version '%s'; " % flake8.__version__ +
            "only version >= 3.0 is supported.", file=sys.stderr)
        return 1

    global flake8_app, StyleGuide
    from flake8.main import application as flake8_app
    from flake8.api.legacy import StyleGuide

    config_file = os.path.join(
        os.path.dirname(__file__), 'configuration', 'ament_flake8.ini')

    parser = argparse.ArgumentParser(
        description='Check code using flake8.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--config',
        metavar='path',
        default=config_file,
        dest='config_file',
        help='The config file')
    parser.add_argument(
        '--linelength', metavar='N', type=int,
        help='The maximum line length (default: specified in the config file)')
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help="The files or directories to check. For directories files ending "
             "in '.py' will be considered.")
    parser.add_argument(
        '--exclude',
        metavar='filename',
        nargs='*',
        dest='excludes',
        help='The filenames to exclude.')
    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    args = parser.parse_args(argv)

    if args.xunit_file:
        start_time = time.time()

    if not os.path.exists(args.config_file):
        print("Could not find config file '%s'" % args.config_file, file=sys.stderr)
        return 1

    report = generate_flake8_report(
        args.config_file, args.paths, args.excludes,
        max_line_length=args.linelength)

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
        print('%d errors' % (report.total_errors))
        print('%d files checked' % len(report.files))

        print('')
        error_type_counts = report.get_error_type_counts()
        for k, v in error_type_counts.items():
            print("'%s'-type errors: %d" % (k, v))

        rc = 1

    # generate xunit file
    if args.xunit_file:
        folder_name = os.path.basename(os.path.dirname(args.xunit_file))
        file_name = os.path.basename(args.xunit_file)
        suffix = '.xml'
        if file_name.endswith(suffix):
            file_name = file_name[0:-len(suffix)]
            suffix = '.xunit'
            if file_name.endswith(suffix):
                file_name = file_name[0:-len(suffix)]
        testname = '%s.%s' % (folder_name, file_name)

        xml = get_xunit_content(report, testname, time.time() - start_time)
        path = os.path.dirname(os.path.abspath(args.xunit_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.xunit_file, 'w') as f:
            f.write(xml)

    return rc


def get_flake8_style_guide(argv):
    application = flake8_app.Application()
    application.find_plugins()
    application.register_plugin_options()
    application.parse_configuration_and_cli(argv)
    application.make_formatter()
    application.make_notifier()
    application.make_guide()
    application.make_file_checker_manager()
    return StyleGuide(application)


def generate_flake8_report(config_file, paths, excludes, max_line_length=None):
    flake8_argv = []
    flake8_argv.append('--config={0}'.format(config_file))
    flake8_argv.append('--exclude={0}'.format(excludes))

    if max_line_length is not None:
        flake8_argv.append('--max-line-length={0}'.format(max_line_length))

    style = get_flake8_style_guide(flake8_argv)

    # Monkey patch formatter to collect all errors
    format_func = style._application.formatter.format
    report = CustomReport()

    def custom_format(error):
        format_func(error)
        report.add_error(error)
        print('')
        print(
            '%s:%d:%d: %s %s' % (
                error.filename, error.line_number,
                error.column_number, error.code, error.text))
    style._application.formatter.format = custom_format

    # Get the names of files checked
    report.report = style.check_files(paths)
    file_checkers = style._application.file_checker_manager.checkers
    report.files = [file_checker.filename for file_checker in file_checkers]

    assert report.report.total_errors == len(report.errors)
    return report


def get_xunit_content(report, testname, elapsed):
    data = {
        'testname': testname,
        'test_count': max(report.total_errors, 1),
        'error_count': report.total_errors,
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

    if report.errors:
        # report each flake8 error/warning as a failing testcase
        for error in report.errors:
            data = {
                'quoted_name': quoteattr(
                    '%s (%s:%d:%d)' % (
                        error.code, error.filename, error.line_number,
                        error.column_number)),
                'testname': testname,
                'quoted_message': quoteattr(
                    '%s:\n%s' % (error.text, error.physical_line)),
            }
            xml += '''  <testcase
    name=%(quoted_name)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s/>
  </testcase>
''' % data

    else:
        # if there are no flake8 problems report a single successful test
        data = {
            'testname': testname,
        }
        xml += '''  <testcase
    name="flake8"
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


class CustomReport(object):

    def __init__(self):
        self.files = []
        self.errors = []
        self.report = None

    @property
    def total_errors(self):
        return len(self.errors)

    def add_error(self, error):
        self.errors.append(error)

    def get_error_type_counts(self):
        error_codes = [e.code for e in self.errors]

        # Determine the type of error by the first character in its error code
        # e.g. 'E261' is type 'E'
        error_types = sorted(set([e[0] for e in error_codes]))

        # Create dictionary of error code types and their counts
        error_type_counts = {}
        for error_type in error_types:
            error_type_counts[error_type] = len([
                e for e in error_codes if e.startswith(error_type)])
        return error_type_counts

    def print_statistics(self):
        self.report._application.report_statistics()


if __name__ == '__main__':
    sys.exit(main())