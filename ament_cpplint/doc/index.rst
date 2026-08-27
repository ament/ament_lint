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

Patches for cpplint 1.6.1. Fixes 1 and 2 have been made upstream.

1. Run with `python3` not `python`. [See](https://github.com/ament/ament_lint/pull/39/files).

```python
#!/usr/bin/env python3
```

2. Don't run `cpp` lints on `.c` files. [See](https://github.com/ament/ament_lint/pull/32).

```python
  # Don't warn in C files about C-style casts
  if os.path.splitext(filename)[1] in ['.c', '.h']:
    return False
```

3. allow using-directive for user-defined literals namespaces. [see](https://github.com/ament/ament_lint/pull/67)

```python
  # Check for 'using namespace' which pollutes namespaces.
  # This is tricky. Although in general 'using namespace' is a Bad Thing,
  # an exception is made for certain standard namespaces, like std::*literals
  # and std::placeholders, which are intended to be used in this fashion.
  # This whitelist may grow over time as needed if/when shiny new libraries
  # come along that are well-behaved in a 'using namespace' context.
  # For example, 'using namespace std::chrono_literals;' is allowed, but
  # 'using namespace foo;' is not allowed.
  # Note that headers are not permitted to use this exception.
  match = Search(r'\busing namespace\s+((\w|::)+)', line)
  if match:
    whitelist = [
      'std::chrono_literals',
      'std::complex_literals',
      'std::literals',
      'std::literals::chrono_literals',
      'std::literals::complex_literals',
      'std::literals::string_literals',
      'std::placeholders',
      'std::string_literals',
    ]
    if IsHeaderExtension(file_extension) or match.group(1) not in whitelist:
      error(filename, linenum, 'build/namespaces', 5,
            'Do not use namespace using-directives.  '
            'Use using-declarations instead.')
```