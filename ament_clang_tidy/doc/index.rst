ament_clang_tidy
==================

Checks the code style of C / C++ source files using `Clang-Tidy
<http://clang.llvm.org/extra/clang-tidy/>`_.
Files with the following extensions are considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``.


How to run the check from the command line?
-------------------------------------------

*Prerequisites*: ``clang-tidy-6.0``, ``clang-tools-6.0``, and ``python-yaml`` packages should
have already been installed. ``compile_commands.json`` files should have already been generated
(e.g.: workspace built with ``colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON``).

.. code:: sh

    ament_clang_tidy [<path> ...]

If ``<path>`` is a directory, it will be recursively searched for
"compile_commands.json" files (this is usually the ``build`` directory of a
``colcon`` workspace). If ``<path>`` is a file, it will be treated as a
"compile_commands.json" file.

The ``--explain-config`` option will explain the origin of the enabled
configuration checks.

The ``--export-fixes`` option will generate a DAT file of the recorded
fixes when supplied with a file name.

When using the option ``--fix-errors`` the proposed changes are
applied in place.

The ``--header-filter`` option will accept a regex and display errors from
the specified non-system header files.  To display errors from all non-system
header, use ``--header-filter='.*'``.

The ``--quiet`` option will suppress printing statistics about ignored
warnings and warnings treated as errors.

The ``--system-headers`` option will display errors from all system header
files.

The ``--xunit-file`` option will generate a xunit compliant XML file when
supplied with a file name.

How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_clang_tidy
<https://github.com/ament/ament_lint>`_.
