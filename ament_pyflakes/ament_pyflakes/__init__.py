import sys
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr

from pyflakes.messages import Message
from pyflakes.reporter import Reporter


def get_xunit_content(report, testname, elapsed):
    test_count = sum([max(len(r[1]), 1) for r in report])
    error_count = sum([len(r[1]) for r in report])
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

    for (filename, errors) in report:

        if errors:
            # report each pyflakes error as a failing testcase
            for error in errors:
                data = {
                    'quoted_name': quoteattr(
                        '%s (%s:%d)' % (
                            type(error).__name__, filename, error.lineno)),
                    'testname': testname,
                    'quoted_message': quoteattr(
                        error.message % error.message_args),
                }
                xml += '''  <testcase
    name=%(quoted_name)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s/>
  </testcase>
''' % data

        else:
            # if there are no pyflakes errors report a single successful test
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
        'escaped_files': escape(''.join(['\n* %s' % r[0] for r in report])),
    }
    xml += '''  <system-out>Checked files:%(escaped_files)s</system-out>
''' % data

    xml += '</testsuite>'
    return xml


class CustomReporter(Reporter):

    def __init__(self):
        super(CustomReporter, self).__init__(sys.stdout, sys.stderr)
        self.errors = []

    def unexpectedError(self, filename, msg):
        self.errors.append(UnexpectedError(filename, msg))

    def syntaxError(self, filename, msg, lineno, offset, text):
        self.errors.append(SyntaxError(filename, msg, lineno, offset, text))

    def flake(self, message):
        self.errors.append(message)


class Location(object):

    def __init__(self, lineno, col_offset=None):
        self.lineno = lineno
        if col_offset is not None:
            self.col_offset = col_offset


class SyntaxError(Message):
    message = 'syntax error %r'

    def __init__(self, filename, msg, lineno, offset, text):
        loc = Location(lineno, col_offset=offset)
        super(SyntaxError, self).__init__(filename, loc)
        self.message_args = (msg, text,)


class UnexpectedError(Message):
    message = 'unexpected error %r'

    def __init__(self, filename, msg):
        loc = Location(0)
        super(UnexpectedError, self).__init__(filename, loc)
        self.message_args = (msg,)
