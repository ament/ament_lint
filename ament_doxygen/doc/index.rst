ament_doxygen
================

Checks that all funtions, variables, classes, methods and attributes in C / C++ code are documented
using `Doxygen <https://www.doxygen.nl/index.html/>`_.

How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_uncrustify Doxyfile


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_doxygen
<https://github.com/ament/ament_lint>`_.
