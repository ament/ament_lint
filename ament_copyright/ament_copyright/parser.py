# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import os
import re

from ament_copyright import ALL_FILETYPES
from ament_copyright import CONTRIBUTING_FILETYPE
from ament_copyright import get_copyright_names
from ament_copyright import get_licenses
from ament_copyright import LICENSE_FILETYPE
from ament_copyright import SOURCE_FILETYPE
from ament_copyright import UNKNOWN_IDENTIFIER


class CopyrightDescriptor:

    def __init__(self, name, year_range):
        self.name = name
        self.year_range = year_range

    def __str__(self):
        s = self.name
        if self.year_range:
            s += ' (%s)' % self.year_range
        return s


class FileDescriptor:

    def __init__(self, filetype, path):
        self.filetype = filetype
        self.path = path
        self.exists = os.path.exists(path)
        self.content = None
        self.license_identifier = UNKNOWN_IDENTIFIER

    def read(self):
        if not self.exists:
            return
        with open(self.path, 'r', encoding='utf-8') as h:
            self.content = h.read()

    def parse(self, allowed_licenses):
        raise NotImplementedError()

    def identify_license(self, content, license_part, licenses=None):
        if content is None:
            return
        if licenses is None:
            licenses = get_licenses()
        formatted_content = remove_formatting(content)

        for name, license_ in licenses.items():
            templates = getattr(license_, license_part)
            for template in templates:
                last_index = -1
                formatted_template = remove_formatting(template)
                template_sections = split_template(formatted_template,
                                                   ['{copyright_holder}', '{copyright}'])
                for license_section in template_sections:
                    # OK, now look for each section of the license in the incoming
                    # content.
                    index = formatted_content.find(license_section.strip())
                    if index == -1 or index <= last_index:
                        # Some part of the license is not in the content, or the license
                        # is rearranged, this license doesn't match.
                        break
                    last_index = index
                else:
                    # We found the license, so set it
                    self.license_identifier = name
                    break


class SourceDescriptor(FileDescriptor):

    def __init__(self, path):
        super(SourceDescriptor, self).__init__(SOURCE_FILETYPE, path)

        self.copyrights = []

        self.copyright_identifiers = []

    def identify_copyright(self):
        known_copyrights = get_copyright_names()
        for c in self.copyrights:
            found_name = c.name
            for identifier, name in known_copyrights.items():
                if name == found_name:
                    self.copyright_identifiers.append(identifier)
                    break
            else:
                self.copyright_identifiers.append(UNKNOWN_IDENTIFIER)

    def parse(self, allowed_licenses):
        self.read()
        if not self.content:
            return

        # skip over coding and shebang lines
        index = scan_past_coding_and_shebang_lines(self.content)
        index = scan_past_empty_lines(self.content, index)

        def parse_comment_block(block):
            copyrights, remaining_block = search_copyright_information(block)
            self.copyrights += copyrights
            # if we haven't found a license yet, try to identify it in this block
            # in case of files with multiple licenses, we only consider the first one found
            # an example is if you copy a file with an existing license and then you prepend yours
            if self.license_identifier == UNKNOWN_IDENTIFIER:
                license_text = '{copyright}' + remaining_block
                self.identify_license(license_text, 'file_headers', allowed_licenses)

        # parse all single-line comment blocks for copyright information
        tmp_index = index
        while True:
            block, tmp_index = get_comment_block(self.content, tmp_index)
            if block:
                parse_comment_block(block)
            else:
                break

        # parse all multi-line comment blocks for copyright information
        tmp_index = index
        while True:
            block, tmp_index = get_multiline_comment_block(self.content, tmp_index)
            if block:
                parse_comment_block(block)
            else:
                break

        self.identify_copyright()


class ContributingDescriptor(FileDescriptor):

    def __init__(self, path):
        super(ContributingDescriptor, self).__init__(CONTRIBUTING_FILETYPE, path)

    def parse(self, allowed_licenses):
        self.read()
        if not self.content:
            return

        self.identify_license(self.content, 'contributing_files', allowed_licenses)


class LicenseDescriptor(FileDescriptor):

    def __init__(self, path):
        super(LicenseDescriptor, self).__init__(LICENSE_FILETYPE, path)

    def parse(self, allowed_licenses):
        self.read()
        if not self.content:
            return

        self.identify_license(self.content, 'license_files', allowed_licenses)


def parse_file(path, allowed_licenses):
    filetype = determine_filetype(path)
    if filetype == SOURCE_FILETYPE:
        d = SourceDescriptor(path)
    elif filetype == CONTRIBUTING_FILETYPE:
        d = ContributingDescriptor(path)
    elif filetype == LICENSE_FILETYPE:
        d = LicenseDescriptor(path)
    else:
        return None
    d.parse(allowed_licenses)
    return d


def determine_filetype(path):
    basename = os.path.basename(path)
    for filetype, filename in ALL_FILETYPES.items():
        if basename == filename:
            return filetype
    return SOURCE_FILETYPE


def get_copyright_information_regex():
    # regex for matching years or year ranges (yyyy-yyyy) separated by colons
    year = r'\d{4}'
    year_range = '%s-%s' % (year, year)
    year_or_year_range = '(?:%s|%s)' % (year, year_range)
    pattern = r'^[^\n\r]?\s*(?:\\copyright\s*)?' \
              r'copyright(?:\s+\(c\))?\s+(%s(?:,\s*%s)*),?\s+([^\n\r]+)$' % \
        (year_or_year_range, year_or_year_range)
    regex = re.compile(pattern, re.DOTALL | re.MULTILINE | re.IGNORECASE)
    return regex


def search_copyright_information(content):
    if content is None:
        return [], content
    regex = get_copyright_information_regex()

    copyrights = []
    while True:
        match = regex.search(content)
        if not match:
            break
        years_span, name_span = match.span(1), match.span(2)
        years = content[years_span[0]:years_span[1]]
        name = content[name_span[0]:name_span[1]]
        copyrights.append(CopyrightDescriptor(name, years))
        content = content[name_span[1]:]

    return copyrights, content


def scan_past_coding_and_shebang_lines(content):
    index = 0
    while (
        is_comment_line(content, index) and
        (is_coding_line(content, index) or
         is_shebang_line(content, index))
    ):
        index = get_index_of_next_line(content, index)
    return index


def get_index_of_next_line(content, index):
    index_n = content.find('\n', index)
    index_r = content.find('\r', index)
    index_rn = content.find('\r\n', index)
    indices = set()
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
    # skip over optional BOM
    if index == 0 and content[0] == '\ufeff':
        index = 1
    return content[index] == '#' or content[index:index + 1] == '//'


def is_coding_line(content, index):
    end_index = get_index_of_next_line(content, index)
    line = content[index:end_index]
    return 'coding=' in line or 'coding:' in line


def is_shebang_line(content, index):
    # skip over optional BOM
    if index == 0 and content[0] == '\ufeff':
        index = 1
    return content[index:index + 2] == '#!'


def get_comment_block(content, index):
    # regex for matching the beginning of the first comment
    # check for doxygen comments (///) before regular comments (//)
    pattern = '^(#|///|//)'
    # also accept BOM if present
    if index == 0 and content[0] == '\ufeff':
        pattern = pattern[0] + '\ufeff' + pattern[1:]
    regex = re.compile(pattern, re.MULTILINE)

    match = regex.search(content, index)
    if not match:
        return None, None
    comment_token = match.group(1)
    start_index = match.start(1)

    end_index = start_index
    while True:
        end_index = get_index_of_next_line(content, end_index)
        if content[end_index:end_index + len(comment_token)] != comment_token:
            break

    block = content[start_index:end_index]
    lines = block.splitlines()
    lines = [line[len(comment_token) + 1:] for line in lines]

    return '\n'.join(lines), start_index + len(comment_token) + 1


def get_multiline_comment_block(content, index):
    patterns = [('^(/[*])', '([*]/)$'),
                ('^(<!--)', '(-->)$')]
    for pattern_pair in patterns:
        start_pattern, end_pattern = pattern_pair
        # find the first match of the comment start token
        # also accept BOM if present
        if index == 0 and content[0] == '\ufeff':
            start_pattern = start_pattern[0] + '\ufeff' + start_pattern[1:]
        start_regex = re.compile(start_pattern, re.MULTILINE)
        start_match = start_regex.search(content, index)
        if not start_match:
            continue
        comment_token = start_match.group(1)
        start_index = start_match.start(1)

        # find the first match of the comment end token
        end_regex = re.compile(end_pattern, re.MULTILINE)
        end_match = end_regex.search(content, index)
        if not end_match:
            continue
        end_index = end_match.start(1)

        # collect all lines between start and end (open interval) and strip out any common prefix
        block = content[start_index:end_index]
        block_lines = block.splitlines()
        if len(block_lines) == 1:
            prefixed_lines = block_lines
        elif len(block_lines) == 2:
            prefixed_lines = block_lines[1:]
        else:
            prefixed_lines = block_lines[1:-1]

        if len(prefixed_lines) > 1:
            line_prefix = os.path.commonprefix(prefixed_lines)
            lines = [line[len(line_prefix):] for line in prefixed_lines]
        else:
            # Single-line header does not have a common prefix to strip out
            lines = prefixed_lines

        return '\n'.join(lines), start_index + len(comment_token) + 1
    return None, index


def scan_past_empty_lines(content, index):
    while is_empty_line(content, index):
        index = get_index_of_next_line(content, index)
    return index


def is_empty_line(content, index):
    return get_index_of_next_line(content, index) == index + 1


def remove_formatting(text):
    return ' '.join(filter(None, text.split()))


# Flat list of sections split on all separators provided
def split_template(sections, separators):
    if type(sections) != list:
        return split_template([sections], separators)
    elif len(separators) > 1:
        return sum([split_template([section], separators[0:1]) for section
                    in sum([split_template([section], separators[1:])
                            for section in sections], [])], [])
    else:
        return list(filter(lambda s: len(s) > 0,
                           sum([section.split(separators[0]) for section in sections], [])))
