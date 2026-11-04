ament_yamllint
==============

Checks YAML files using `yamllint <https://yamllint.readthedocs.io>`_.
Files with the following extensions are being considered: ``.yaml``, ``.yml``.


How to run the check from the command line?
-------------------------------------------

The command line tool is provided by the package `ament_yamllint
<https://github.com/ament/ament_lint>`_.


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

``package.xml``:

.. code:: xml

    <buildtool_depend>ament_cmake</buildtool_depend>
    <test_depend>ament_cmake_yamllint</test_depend>

``CMakeLists.txt``:

.. code:: cmake

    find_package(ament_cmake REQUIRED)
    if(BUILD_TESTING)
      find_package(ament_cmake_yamllint REQUIRED)
      ament_yamllint()
    endif()

The documentation of the package `ament_cmake_test
<https://github.com/ament/ament_cmake>`_ provides more information on testing
in CMake ament packages.
