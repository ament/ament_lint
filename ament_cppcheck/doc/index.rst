ament_cppcheck
==============

Performs a static code analysis of C / C++ source files using `CppCheck
<http://cppcheck.sourceforge.net/>`_.
Files with the following extensions are being considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_cppcheck [<path> ...]


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

``package.xml``:

.. code:: xml

    <build_depend>ament_cmake_test</build_depend>
    <test_depend>ament_cppcheck</test_depend>

``CMakeLists.txt``:

.. code:: cmake

    find_package(ament_cmake_test REQUIRED)
    if(AMENT_ENABLE_TESTING)
      find_package(ament_cppcheck REQUIRED)
      ament_cppcheck()
    endif()

When running multiple linters as part of the CMake tests the documentation of
the package `ament_lint_auto <https://github.com/ament/ament_lint>`_ might
contain some useful information.

The documentation of the package `ament_cmake_test
<https://github.com/ament/ament_cmake>`_ provides more information on testing
in CMake ament packages.
