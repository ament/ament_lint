ament_cobra
===========

Analyzes ``.c``, ``.cc``, ``.cpp``, and ``.cxx`` files using the `Cobra <https://spinroot.com/uno/>`_ static analyzer.


Running from the command line
-----------------------------

The ament_cobra tool has the following command line:

.. code:: sh

    usage: ament_cobra [-h] [--include_dirs [INCLUDE_DIRS [INCLUDE_DIRS ...]]] [--exclude [EXCLUDE [EXCLUDE ...]]]
                       [--ruleset RULESET] [--compile_cmds COMPILE_CMDS] [--xunit-file XUNIT_FILE] [--cobra-version]
                       [paths [paths ...]]

    Analyze source code using the cobra static analyzer.

    positional arguments:
      paths                 Files and/or directories to be checked. Directories are searched recursively for files ending
                            in one of '.c', '.cc', '.cpp', '.cxx'. (default: ['.'])

    optional arguments:
      -h, --help            show this help message and exit
      --include_dirs [INCLUDE_DIRS [INCLUDE_DIRS ...]]
                            Include directories for C/C++ files being checked.Each directory is passed to cobra as
                            '-I<include_dir>' (default: None)
      --exclude [EXCLUDE [EXCLUDE ...]]
                            Exclude C/C++ files from being checked. (default: [])
      --ruleset RULESET     The cobra rule set to use to analyze the code: basic, cwe, p10, jpl, or misra2012. (default:
                            basic)
      --compile_cmds COMPILE_CMDS
                            The compile_commands.json file from which to gather preprocessor directives. This option will
                            take precedence over the --include_dirs options and any directories specified using
                            --include_dirs will be ignored. Instead, ament_cobra will gather all preprocessor options from
                            the compile_commands.json file. (default: None)
      --xunit-file XUNIT_FILE
                            Generate a xunit compliant XML file (default: None)
      --cobra-version       Get the cobra version, print it, and then exit (default: False)
      --verbose             Display verbose output (default: False)

Adding ament_cobra to unit tests
--------------------------------

CMake integration is provided by the package `ament_cmake_cobra
<https://github.com/ament/ament_lint>`_.
