ament_clang_tidy
==================

Checks the code style of C / C++ source files using `Clang-Tidy
<http://clang.llvm.org/extra/clang-tidy/>`_.
Files with the following extensions are considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_clang_tidy [<path> ...]

When using the option ``--fix-errors`` the proposed changes are
applied in place.

The ``--explain-config`` option will explain the origin of the enabled
configuration checks.

The ``--header-filter`` option will accept a regex and display errors from
the specified non-system header files.  To display errors from all non-system
header, use ``--header-filter='.*'``.

The ``--system-headers`` option will display errors from all system header
files.

The ``--quiet`` option will suppress printing statistics about ignored
warnings and warnings treated as errors.

The ``--export-fixes`` option will generate a DAT file of the recorded
fixes when supplied with a file name.

The ``--xunit-file`` option will generate a xunit compliant XML file when
supplied with a file name.

How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_clang_tidy
<https://github.com/ament/ament_lint>`_.
