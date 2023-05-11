ament_bandit
==============

Performs a static code analysis of Python source files using `Bandit
<https://pypi.org/project/bandit/>`_.
Files with the following extensions are being considered:
``.py``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_bandit [<path> ...]


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_bandit
<https://github.com/ament/ament_lint>`_.
