ament_cmake_clang_format
========================

Checks the code style of C / C++ source files using `ClangFormat
<http://clang.llvm.org/docs/ClangFormat.html>`_.
Files with the following extensions are being considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``.


How to run the check from the command line?
-------------------------------------------

The command line tool is provided by the package `ament_clang_format
<https://github.com/ament/ament_lint>`_.


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

``package.xml``:

.. code:: xml

    <buildtool_depend>ament_cmake</buildtool_depend>
    <test_depend>ament_cmake_clang_format</test_depend>

``CMakeLists.txt``:

.. code:: cmake

    find_package(ament_cmake REQUIRED)
    if(BUILD_TESTING)
      find_package(ament_cmake_clang_format REQUIRED)
      ament_clang_format()
    endif()

To override the `clang-format` version to be used, specify the
`CLANG_FORMAT_VERSION` argument to `ament_clang_format()`:

.. code:: cmake

    find_package(ament_cmake REQUIRED)
    if(BUILD_TESTING)
      find_package(ament_cmake_clang_format REQUIRED)
      ament_clang_format(CLANG_FORMAT_VERSION 10)
    endif()

Alternatively, this can be specified by defining the
`ament_cmake_clang_format_CLANG_FORMAT_VERSION` global CMake variable. This can
be useful when the test is run indirectly, such as through `ament_lint_auto`:

.. code:: cmake

    find_package(ament_cmake REQUIRED)
    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_clang_format_CLANG_FORMAT_VERSION 10)
    ament_lint_auto_find_test_dependencies()
    endif()

The `clang-format` configuration file can be overridden in the same way using
either the `CONFIG_FILE` argument or the `ament_cmake_clang_format_CONFIG_FILE`
global variable.

When running multiple linters as part of the CMake tests the documentation of
the package `ament_lint_auto <https://github.com/ament/ament_lint>`_ might
contain some useful information.

The documentation of the package `ament_cmake_test
<https://github.com/ament/ament_cmake>`_ provides more information on testing
in CMake ament packages.
