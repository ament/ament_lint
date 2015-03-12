ament_copyright
===============

Checks C / C++ / CMake / Python source files for the existance of a copyright
notice.
Files with the following extensions are being considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``,
``.cmake``, ``.py``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_copyright [<path> ...]

When using the option ``--list-names`` a list of known copyright holders is
shown.

When using the option
``--add-missing <NAME | "Copyright holder string">`` a copyright notice is
added to all files which lack one.
The argument can either be a name from the list returned by ``--list-names`` or
a custom string.

When using the option ``--add-copyright-year`` existing copyright notices are
being updated to include the current year.


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_copyright
<https://github.com/ament/ament_lint>`_.
