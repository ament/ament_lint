ament_lint_auto
===============

The package simplifies using multiple linters as part of the CMake tests.
It reduces the amount of CMake code to a bare minimum.

``CMakeLists.txt``:

.. code:: cmake

    # this must happen before the invocation of ament_package()
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
    endif()

The set of linters to be used is then only specified in the package manifest as
test dependencies.

``package.xml``:

.. code:: xml

    <test_depend>ament_lint_auto</test_depend>

    <!-- add test dependencies on any linter, e.g. -->
    <test_depend>ament_clang_format</test_depend>
    <test_depend>ament_cppcheck</test_depend>
    <test_depend>ament_pycodestyle</test_depend>

Since recursive dependencies are also being used a single test dependency is
sufficient to test with a set of common linters.

``package.xml``:

.. code:: xml

    <test_depend>ament_lint_auto</test_depend>

    <!-- this recursively depends on a set of common linters -->
    <test_depend>ament_lint_common</test_depend>

The documentation of the package `ament_cmake_test
<https://github.com/ament/ament_cmake>`_ provides more information on testing
in CMake ament packages.
