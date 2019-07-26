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

The ``--add-headers`` option will display errors from all non-system
headers.

How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_clang_tidy
<https://github.com/ament/ament_lint>`_.
