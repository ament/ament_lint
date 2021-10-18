ament_cobra
===========

Performs a static code analysis on source files using `Cobra
<https://spinroot.com/uno/>`_.
Files with the following extensions are being considered:
``.c``, ``.h``


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_cobra [<path> ...]


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_cobra
<https://github.com/ament/ament_lint>`_.
