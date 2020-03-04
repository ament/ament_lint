ament_pycodestyle
=================

Checks the code style of Python source files using `pycodestyle
<http://pycodestyle.readthedocs.org/>`_.
Files with the following extensions are being considered: ``.py``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_pycodestyle [<path> ...]


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_pycodestile
<https://github.com/ament/ament_lint>`_.
