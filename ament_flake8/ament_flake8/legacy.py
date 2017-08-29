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

from distutils.version import LooseVersion

import flake8

# Ensure that importing this module doesn't fail when imported on systems with flake8>=3.0.
# This can happen during coverage tests on systems with flake8>=3.0, for example.
if LooseVersion(flake8.__version__) < '3.0':
    import flake8.engine
    from flake8.reporter import QueueReport

    class CustomReport(QueueReport):

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
                'source_line': line.splitlines()[0] if line else '',
            })
            return code

        def get_error_codes(self):
            return [e['error_code'] for e in self.errors]

    class CustomStyleGuide(flake8.engine.NoQAStyleGuide):

        def input_file(self, filename, **kwargs):
            self.options.reporter.files.append(filename)
            return super(CustomStyleGuide, self).input_file(filename, **kwargs)

    def generate_flake8_report(config_file, paths, excludes, max_line_length=None):
        kwargs = {
            'repeat': True,
            'show_source': True,
            'verbose': True,
            'reporter': CustomReport,
            'config_file': config_file,
            'jobs': 1,
        }
        if max_line_length is not None:
            kwargs['max_line_length'] = max_line_length

        # add options for flake8 plugins
        kwargs['parser'], options_hooks = flake8.engine.get_parser()
        flake8style = CustomStyleGuide(**kwargs)
        options = flake8style.options
        for options_hook in options_hooks:
            options_hook(options)

        if excludes:
            flake8style.options.exclude += excludes

        # flake8 uses a wrapper StyleGuide to handle some particular OSErrors
        kwargs['styleguide'] = flake8style
        wrapper_style_guide = flake8.engine.StyleGuide(**kwargs)

        return wrapper_style_guide.check_files(paths)
