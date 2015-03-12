ament_clang_format
==================

Checks the code style of C / C++ source files using `ClangFormat
<http://clang.llvm.org/docs/ClangFormat.html>`_.
Files with the following extensions are being considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_clang_format [<path> ...]

When using the option ``--reformat`` the proposed changes are applied in place.


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

``package.xml``:

.. code:: xml

    <build_depend>ament_cmake_test</build_depend>
    <test_depend>ament_clang_format</test_depend>

``CMakeLists.txt``:

.. code:: cmake

    find_package(ament_cmake_test REQUIRED)
    if(AMENT_ENABLE_TESTING)
      find_package(ament_clang_format REQUIRED)
      ament_clang_format()
    endif()

When running multiple linters as part of the CMake tests the documentation of
the package `ament_lint_auto <https://github.com/ament/ament_lint>`_ might
contain some useful information.

The documentation of the package `ament_cmake_test
<https://github.com/ament/ament_cmake>`_ provides more information on testing
in CMake ament packages.
