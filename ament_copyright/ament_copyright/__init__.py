# Copyright 2014 Open Source Robotics Foundation, Inc.

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
import pkg_resources
import re
import sys
import time
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr

AMENT_COPYRIGHT_GROUP = 'ament_copyright.names'


def main(argv=sys.argv[1:]):
    extensions = [
        'c', 'cc', 'cpp', 'cxx', 'h', 'hh', 'hpp', 'hxx',
        'cmake',
        'py',
    ]

    parser = argparse.ArgumentParser(
        description='Check code for existance of a copyright reference.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help="The files or directories to check. For directories files ending "
             'in %s will be considered.' %
             ', '.join(["'.%s'" % e for e in extensions]))
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--add-missing',
        help='Add missing copyright notice with the passed copyright holder')
    group.add_argument(
        '--add-copyright-year',
        action='store_true',
        help='Add the current year to existing copyright notices')
    group.add_argument(
        '--list-names',
        action='store_true',
        help='List name of possible copyright holders')
    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    group.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    args = parser.parse_args(argv)

    names = get_copyright_names()
    if args.list_names:
        for key in sorted(names.keys()):
            print('%s: %s' % (key, names[key]))
        return 0

    if args.xunit_file:
        start_time = time.time()

    filenames = get_files(args.paths, extensions)
    if not filenames:
        print('No files found', file=sys.stderr)
        return 0

    if args.add_missing:
        name = names.get(args.add_missing, args.add_missing)
        add_missing(filenames, name)
        return 0

    if args.add_copyright_year:
        add_copyright_year(filenames)
        return 0

    report = []

    # check each file for copyright
    for filename in filenames:
        print(filename)
        has_copyright = check_file_for_copyright(filename)
        if not has_copyright:
            print('%s: could not find copyright reference' % filename,
                  file=sys.stderr)
        report.append((filename, has_copyright))
        print('')

   # output summary
    error_count = len([r for r in report if not r[1]])
    if not error_count:
        print('No errors')
        rc = 0
    else:
        print('%d errors' % error_count, file=sys.stderr)
        rc = 1

    # generate xunit file
    if args.xunit_file:
        folder_name = os.path.basename(os.path.dirname(args.xunit_file))
        file_name = os.path.basename(args.xunit_file)
        suffix = '.xml'
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


def get_copyright_names():
    names = {}
    for entry_point in pkg_resources.iter_entry_points(
            group=AMENT_COPYRIGHT_GROUP):
        name = entry_point.load()
        names[entry_point.name] = name
    return names


def get_files(paths, extensions):
    files = []
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    _, ext = os.path.splitext(filename)
                    if ext in ['.%s' % e for e in extensions]:
                        files.append(os.path.join(dirpath, filename))
        if os.path.isfile(path):
            files.append(path)
    return files


def add_missing(filenames, name):
    copyright = 'Copyright %s %s' % (time.strftime('%Y'), name)
    print('Adding the following copyright notice:')
    print(' ', copyright)
    print('To the following file:')

    for filename in filenames:
        has_copyright = check_file_for_copyright(filename)
        if has_copyright:
            continue

        print('*', filename)
        add_copyright(filename, copyright)


def add_copyright_year(filenames):
    current_year = time.strftime('%Y')
    print('Adding the current year to existing copyright notices:')

    # regex for matching years or year ranges (yyyy-yyyy) separated by colons
    year = '\d{4}'
    year_range = '%s-%s' % (year, year)
    year_or_year_range = '(%s|%s)' % (year, year_range)
    pattern = '(\s+%s(\s*,\s*%s)*)\s+.+' % \
        (year_or_year_range, year_or_year_range)
    regex = re.compile(pattern)

    for filename in filenames:
        has_copyright = check_file_for_copyright(filename)
        if not has_copyright:
            continue

        with open(filename, 'r') as h:
            content = h.read()

        index = scan_past_coding_and_shebang_lines(content)
        index = scan_past_empty_lines(content, index)

        assert is_copyright_line(content, index), \
            "Could not find copyright line in file '%s'" % filename

        # extract copyright years
        end_index = get_index_of_next_line(content, index)
        line = content[index:end_index]

        copyright = 'copyright'
        copyright_index = line.lower().index(copyright)
        rest_line = line[copyright_index + len(copyright):]

        match = regex.match(rest_line)
        if not match:
            print("Could not find copyright year in file '%s' in line [%s]" %
                  (filename, line), file=sys.stderr)
            continue
        copyright_year = match.group(1)

        # skip if copyright year is already up-to-date
        if current_year in copyright_year:
            continue

        end_index = index + copyright_index + len(copyright) + \
            len(copyright_year)
        if copyright_year[-4:] != str(int(current_year) - 1):
            # append current year
            content = content[:end_index] + ', ' + current_year + \
                content[end_index:]
        elif copyright_year[-5:-4] == '-':
            # update end year of range
            content = content[:end_index - 4] + current_year + \
                content[end_index:]
        else:
            # update year to be a range
            content = content[:end_index] + '-' + current_year + \
                content[end_index:]

        # output beginning of file for debugging
        # index = end_index
        # for _ in range(3):
        #     index = get_index_of_next_line(content, index)
        # print('<<<')
        # print(content[:index - 1])
        # print('>>>')

        with open(filename, 'w') as h:
            h.write(content)


def add_copyright(filename, copyright):
    """
    Add the copyright message to a file.

    The copyright is placed below an optional shebang line as well as an
    optional coding line.
    It is preceded by an empty line if not at the beginning of the file and
    always followed by an empty line.
    """
    with open(filename, 'r') as h:
        content = h.read()

    begin_index = scan_past_coding_and_shebang_lines(content)
    end_index = scan_past_empty_lines(content, begin_index)

    # inject copyright message
    comment = get_comment(filename, copyright)
    inserted_block = '%s\n\n' % comment
    if begin_index > 0:
        inserted_block = '\n' + inserted_block
    content = content[:begin_index] + inserted_block + content[end_index:]

    # output beginning of file for debugging
    # index = end_index + len(inserted_block)
    # for _ in range(3):
    #     index = get_index_of_next_line(content, index)
    # print('<<<')
    # print(content[:index - 1])
    # print('>>>')

    with open(filename, 'w') as h:
        h.write(content)


def scan_past_coding_and_shebang_lines(content):
    index = 0
    while (
        is_comment_line(content, index) and
        (is_coding_line(content, index) or
         is_shebang_line(content, index))
    ):
        index = get_index_of_next_line(content, index)
    return index


def scan_past_empty_lines(content, index):
    while is_empty_line(content, index):
        index = get_index_of_next_line(content, index)
    return index


def get_index_of_next_line(content, index):
    index_n = content.find('\n', index)
    index_r = content.find('\r', index)
    index_rn = content.find('\r\n', index)
    indices = set([])
    if index_n != -1:
        indices.add(index_n)
    if index_r != -1:
        indices.add(index_r)
    if index_rn != -1:
        indices.add(index_rn)
    if not indices:
        return len(content)
    index = min(indices)
    if index == index_rn:
        return index + 2
    return index + 1


def is_comment_line(content, index):
    return content[index] == '#' or content[index:index + 1] == '//'


def is_coding_line(content, index):
    end_index = get_index_of_next_line(content, index)
    line = content[index:end_index]
    return 'coding=' in line or 'coding:' in line


def is_shebang_line(content, index):
    return content[index:index + 1] == '#!'


def is_empty_line(content, index):
    return get_index_of_next_line(content, index) == index + 1


def get_comment(filename, msg):
    if filename.endswith('.py'):
        line_prefix = '# '
    else:
        line_prefix = '// '

    comment = ''
    index = 0
    while True:
        new_index = get_index_of_next_line(msg, index)
        if new_index == index:
            break
        comment += line_prefix + msg[index:new_index]
        index = new_index
    return comment


def check_file_for_copyright(filename):
    with open(filename, 'r') as h:
        content = h.read()

    index = scan_past_coding_and_shebang_lines(content)
    index = scan_past_empty_lines(content, index)

    return is_copyright_line(content, index)


def is_copyright_line(content, index):
    if not is_comment_line(content, index):
        return False

    end_index = get_index_of_next_line(content, index)
    line = content[index:end_index].lower()

    if line.startswith('#'):
        line = line[1:]
    elif line.startswith('//'):
        line = line[2:]
    else:
        assert False, 'Unknown comment line [%s]' % line

    return line.lstrip().lower().startswith('copyright')


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
