# ament_bandit

Analyzes ``.py`` files using the [Bandit](https://bandit.readthedocs.io/) static analyzer.

## Install

`cd ament_bandit`

`python3 setup.py install`

## Quick test

`ament_bandit`

Runs Bandit recursively on the current directory. If run without specifying output format, prints the errors found and error count to the screen. 

### Examples

Get output file in `json` format:

`ament_bandit -f=json -o=bandit-results.json`

Get `xunit` formatted results:

`ament_bandit --xunit-file xunit-results.xml`


## Running from the command line

The ament_bandit tool has the following command line options:

    usage: ament_bandit [-h] [-f FORMAT] [-o OUTPUT] [-x [EXCLUDE [EXCLUDE ...]]] 
            [--xunit-file XUNIT_FILE] [--bandit-version] [paths [paths ...]]

    Analyze source code using the Bandit static analyzer.

    positional arguments:
      paths                 Files and/or directories to be checked. Directories are searched 
                            recursively for files ending in '.py'. (default: ['.'])

    optional arguments:
      -h, --help            Show this help message and exit
      -f --format           Specify output format {csv,custom,html,json,screen,txt,xml,yaml} 
                            (default: None)
      -o --output           Write report to filename. (default: None)
      -x [EXCLUDE [EXCLUDE ...]], --exclude [EXCLUDE [EXCLUDE ...]]
                            Exclude files from being checked. (default: None)
      --xunit-file XUNIT_FILE
                            Generate a xunit compliant XML file (default: None)
      --bandit-version      Get the Bandit version, print it, and then exit

## Adding ament_bandit to unit tests

CMake integration is provided by the package [ament_cmake_bandit](https://github.com/ament/ament_lint).
