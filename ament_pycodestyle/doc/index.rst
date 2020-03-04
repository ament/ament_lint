ament_pep8
==========

Checks the code style of Python source files using `pep8
<http://pep8.readthedocs.org/>`_.
Files with the following extensions are being considered: ``.py``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_pep8 [<path> ...]


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_pep8
<https://github.com/ament/ament_lint>`_.
