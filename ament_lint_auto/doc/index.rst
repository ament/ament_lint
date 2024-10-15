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
    <test_depend>ament_cmake_clang_format</test_depend>
    <test_depend>ament_cmake_cppcheck</test_depend>
    <test_depend>ament_cmake_pycodestyle</test_depend>

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


How to make ``ament_lint_auto`` work with linter macros?
--------------------------------------------------------
Lets consider the following code snippet:

``CMakeLists.txt``:

.. code:: cmake

    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_lint_common REQUIRED)
    ament_cpplint(
        MAX_LINE_LENGTH 90
    )
    # This code snippet will fail

By default, this code fails, as ``ament_cpplint`` macro is invoked twice - once directly in ``CMakeLists.txt`` file, and once by the ``ament_lint_auto`` hook.

Setting the ``AMENT_LINT_AUTO_SKIP_PREEXISTING_TESTS`` flag allows ``ament_lint_auto`` to skip linters already invoked directly in the ``CMakeLists.txt`` file.

For example:

``CMakeLists.txt``:

.. code:: cmake

    set(AMENT_LINT_AUTO_SKIP_PREEXISTING_TESTS ON)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_lint_common REQUIRED)
    ament_cpplint(
        MAX_LINE_LENGTH 90
    )
    # This code snippet works, and settings from the CMakeLists.txt are passed to ``cpplint`` linter.

Possible use cases for this flag include:

**1. Custom Linter Settings**

``AMENT_LINT_AUTO_SKIP_PREEXISTING_TESTS`` allows to directly call a linter macro with specific settings not supported by ``ament_lint_auto`` hooks, while using ``ament_lint_common`` for remaining linters.

**2. Common Linter Set**

The flag makes it possible to declare a common set of linters for all packages in a ROS workspace, regardless of content of ``CMakeLists.txt`` files of packages. 

To declare common set of linters, inject a CMake script during the workspace build, using ``CMAKE_PROJECT_INCLUDE`` build. In that script, enable the flag, and invoke ``ament_lint_auto`` with desired set of linters.

Enabling the flag avoids test re-definition errors for packages that directly call linter macros. This approach is used in SpaceROS to ensure that common set of linters is called for all applicable ROS packages.
