ament_cpplint
==========

Checks the code style of C / C++ source files using `cpplint
<https://github.com/cpplint/cpplint>`_.
Files with the following extensions are being considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_cpplint [<path> ...]


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_cpplint
<https://github.com/ament/ament_lint>`_.

Patches for cpplint 1.6.1.
Note these fixes have been fixed upstream already.

1. Run with `python3` not `python`.

```python
#!/usr/bin/env python3
```

2. Don't run `cpp` lints on `.c` files.

```python
  # Don't warn in C files about C-style casts
  if os.path.splitext(filename)[1] in ['.c', '.h']:
    return False
```