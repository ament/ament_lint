ament_uno
=========

Performs a static code analysis of C source files using `Uno
<https://spinroot.com/uno/>`_.
Files with the following extensions are being considered:
``.c``, ``.h``


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_uno [<path> ...]


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_uno
<https://github.com/ament/ament_lint>`_.
