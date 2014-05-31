import pep8

from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr


def generate_pep8_report(paths):
    pep8style = CustomStyleGuide(
        repeat=True,
        show_source=True,
        verbose=True,
        reporter=CustomReport,
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
                'quoted_location': quoteattr(
                    '%(path)s:%(row)d:%(column)d' % error),
                'error_code': error['error_code'],
                'quoted_message': quoteattr(
                    '%(error_message)s:\n%(source_line)s' % error),
            }
            xml += '''  <testcase
    name=%(quoted_location)s
    classname="%(error_code)s"
  >
      <failure message=%(quoted_message)s/>
  </testcase>
''' % data

    else:
        # if there are no pep8 errors/warnings report a single successful test
        xml += '''  <testcase
    name="ament_pep8"
    status="No errors or warnings"/>
'''

    # output list of checked files
    data = {
        'escaped_files': escape(''.join(['\n* %s' % f for f in report.files])),
    }
    xml += '''  <system-out>Checked files:%(escaped_files)s</system-out>
''' % data

    xml += '</testsuite>'
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
