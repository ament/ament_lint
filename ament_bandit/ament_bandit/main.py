#!/usr/bin/env python3

import argparse
from collections import defaultdict
import os
from shutil import which
import subprocess
import sys
import time
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr
from xml.etree import ElementTree


def get_bandit_version(bandit_bin):
    version_cmd = [bandit_bin, '--version']
    output = subprocess.check_output(version_cmd)

    # parse and validate version number 
    output = output.decode().strip()
    tokens = output.split()

    if tokens[0] != 'bandit':
        raise RuntimeError("unexpected bandit version name '{}'".format(output))

    return tokens[1]

    
def find_bandit_executable():
    additional_paths = None
    if os.name == 'nt':
        # search in location where bandit is installed via chocolatey
        program_files_32 = os.environ.get('ProgramFiles(x86)', 'C:\\Program Files (x86)')
        additional_paths = [os.path.join(program_files_32, 'bandit')]
    return find_executable('bandit', additional_paths=additional_paths)


def find_executable(file_name, additional_paths=None):
    path = None
    if additional_paths:
        path = os.getenv('PATH', os.defpath)
        path += os.path.pathsep + os.path.pathsep.join(additional_paths)
    return which(file_name, path=path)


def get_files(paths, extensions):
    files = []
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                if 'AMENT_IGNORE' in dirnames + filenames:
                    dirnames[:] = []
                    continue
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
    return [os.path.normpath(f) for f in files]


def get_xunit_content(report, testname, elapsed, skip=None):
    test_count = sum(max(len(r), 1) for r in report.values())
    error_count = sum(len(r) for r in report.values())
    data = {
        'testname': testname,
        'test_count': test_count,
        'error_count': error_count,
        'time': '%.3f' % round(elapsed, 3),
        'skip': test_count if skip else 0,
    }
    xml = """<?xml version="1.0" encoding="UTF-8"?>
<testsuite
  name="%(testname)s"
  tests="%(test_count)d"
  errors="0"
  failures="%(error_count)d"
  time="%(time)s"
  skipped="%(skip)d"
>
""" % data

    for filename in sorted(report.keys()):
        errors = report[filename]

        if skip:
            data = {
              'quoted_name': quoteattr(filename),
              'testname': testname,
              'quoted_message': quoteattr(''),
              'skip': skip,
            }
            xml += """  <testcase
    name=%(quoted_name)s
    classname="%(testname)s"
  >
    <skipped type="skip" message=%(quoted_message)s>
      ![CDATA[Test Skipped due to %(skip)s]]
    </skipped>
  </testcase>
""" % data
        elif errors:
            # report each error as a failing testcase
            for error in errors:
                data = {
                    'quoted_name': quoteattr(
                        '%s: %s (%s)' % (
                            error['severity'], error['id'],
                            filename)),
                    'testname': testname,
                    'quoted_message': quoteattr(error['msg']),
                }
                xml += """  <testcase
    name=%(quoted_name)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s/>
  </testcase>
""" % data

        else:
            # if there are no cpplint errors report a single successful test
            data = {
                'quoted_location': quoteattr(filename),
                'testname': testname,
            }
            xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"/>
""" % data

    # output list of checked files
    if skip:
        data = {
            'skip': skip,
        }
        xml += """  <system-err>Tests Skipped due to %(skip)s</system-err>
""" % data
    else:
        data = {
            'escaped_files': escape(
                ''.join(['\n* %s' % r for r in sorted(report.keys())])
            ),
        }
        xml += """  <system-out>Checked files:%(escaped_files)s</system-out>
""" % data

    xml += '</testsuite>\n'
    return xml


def write_xunit_file(xunit_file, report, duration, skip=None):
    folder_name = os.path.basename(os.path.dirname(xunit_file))
    file_name = os.path.basename(xunit_file)
    suffix = '.xml'
    if file_name.endswith(suffix):
        file_name = file_name[0:-len(suffix)]
        suffix = '.xunit'
        if file_name.endswith(suffix):
            file_name = file_name[0:-len(suffix)]
    testname = '%s.%s' % (folder_name, file_name)

    xml = get_xunit_content(report, testname, duration, skip)
    path = os.path.dirname(os.path.abspath(xunit_file))
    if not os.path.exists(path):
        os.makedirs(path)
    with open(xunit_file, 'w') as f:
        f.write(xml)


def main(argv=sys.argv[1:]):
    extensions = ['py']

    parser = argparse.ArgumentParser(
        description='Perform static code analysis using bandit.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help='Files and/or directories to be checked. Directories are searched recursively for '
             'files ending in %s.' %
             ', '.join(["'.%s'" % e for e in extensions]))
    parser.add_argument(
        '-f', '--format',
        help="Specify output format {csv,custom,html,json,screen,txt,xml,yaml}")
    parser.add_argument(
        '-o', '--output',
        help="Write report to filename.")
    parser.add_argument(
        '-x', '--exclude',
        nargs='*',
        help='Exclude files from being checked.')
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    parser.add_argument(
        '--bandit-version',
        action='store_true',
        help='Get the bandit version, print it, and then exit.')
    args = parser.parse_args(argv)

    bandit_bin = find_bandit_executable()
    if not bandit_bin:
        print("Could not find 'bandit' executable", file=sys.stderr)
        return 1

    bandit_version = get_bandit_version(bandit_bin)

    if args.xunit_file:
        start_time = time.time()

    files = get_files(args.paths, extensions)
    if not files:
        print('No files found', file=sys.stderr)
        return 1

    # build bandit command
    cmd = [bandit_bin, '-r']
    cmd.extend(files)
    if args.format:
        cmd.extend(['--format={0}'.format(args.format)])
    if args.output:
        cmd.extend(['--output={0}'.format(args.output)])
    for exclude in (args.exclude or []):
        cmd.extend(['--exclude=' + exclude])
    if args.bandit_version:
        print("Bandit version is:", bandit_version)
        return 0

    # invoke bandit
    try:
        p = subprocess.Popen(cmd, stderr=subprocess.PIPE)
        p.communicate()[1]
    except subprocess.CalledProcessError as e:
        print("The invocation of 'bandit' failed with error code %d: %s" %
              (e.returncode, e), file=sys.stderr)
        return 1

    if args.xunit_file:
        # generate xml file for results report
        try:
            cmd = [bandit_bin, '-r']
            cmd.extend(files)
            cmd.extend(['--format={0}'.format('xml')])
            cmd.extend(['--output={0}'.format('bandit_results.xml')])
            p = subprocess.Popen(cmd, stderr=subprocess.PIPE)
            p.communicate()[1]
            tree = ElementTree.parse('bandit_results.xml')
            root = tree.getroot()
        except ElementTree.ParseError as e:
            print('Invalid XML to parse: %s' % str(e),
                    file=sys.stderr)
            return 1

        # generate error report
        report = defaultdict(list)

        for filename in files:
            report[filename] = []

        for testcase in root.findall('.//testcase'):
            filename = testcase.get('classname')
            error = testcase.find('error')
            data = {
                'id': testcase.get('name'),
                'severity': error.get('type'),
                'msg': error.get('message'),
            }
            for key in report.keys():
                if os.path.samefile(key, filename):
                    filename = key
                    break

            report[filename].append(data)
            data = dict(data)
            data['filename'] = filename

        # print error count
        error_count = sum(len(r) for r in report.values())
        if not error_count:
            print('No problems found')
        else:
            print('%d errors' % error_count, file=sys.stderr)

        # generate xunit file
        write_xunit_file(args.xunit_file, report, time.time() - start_time)

    return 0


if __name__ == '__main__':
    sys.exit(main())
