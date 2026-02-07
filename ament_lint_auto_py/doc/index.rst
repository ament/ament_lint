ament_lint_auto_py
==================

The package simplifies using multiple linters as part of pytest tests.

To have `ament_lint_auto_py` collect and run the linters include it in the `package.xml`

``package.xml``:

.. code:: xml

    <test_depend>ament_lint_auto_py</test_depend>


The set of linters to be used is then only specified in the package manifest as
test dependencies.

``package.xml``:

.. code:: xml

    <test_depend>ament_lint_auto_py</test_depend>

    <!-- add test dependencies on any linter, e.g. -->
    <test_depend>ament_clang_format</test_depend>
    <test_depend>ament_cppcheck</test_depend>
    <test_depend>ament_pycodestyle</test_depend>

Since recursive dependencies are also being used a for packages starting with `ament_lint_*` a single test dependency is
sufficient to test with a set of common linters.

``package.xml``:

.. code:: xml

    <test_depend>ament_lint_auto_py</test_depend>

    <!-- this recursively depends on a set of common linters -->
    <test_depend>ament_lint_common_py</test_depend>


How to exclude linter modules with ament_lint_auto_py?
---------------------------------------------------

Linter modules can be excluded via the pytest configurable variables `ament_lint_auto_exclude` in pytest config files like `pytest.ini` or `pyproject.toml`.

As an example to exclude the `copyright` linter:

.. code::
    [pytest]

    ament_lint_auto_exclude = ament_copyright


How to exclude files with ament_lint_auto_py?
------------------------------------------

Linter hooks shall conform to the ament_lint_auto_py convention of excluding files
specified in the environment list variable `ament_lint_auto_file_exclude`.

.. code::
    [pytest]

    ament_lint_auto_file_exclude = /path/to/ignored_file

For a more specific example, this excludes all python files matching a pattern using globbing.
Multiple expressions can be combined on multiple lines.


.. code::
    [pytest]

    ament_lint_auto_file_exclude = 
        src/*
        test/*.cpp

How to register 3rd party linters with ament_lint_auto_py?
---------------------------------------------------------

To register a third party linter implement class like the following.

.. code:: python

  class CustomRunner:

      NAME = 'ament_custom'
      FILE_TYPES = ('*.py',)

      def __init__(self, args: list[str]) -> None:
          self.args = args

      def __call__(self) -> Literal[0, 1]:
          return main(self.args)

  def main():
    # Custom linting
    pass

Then in a `setup.py` register an entry point for the `CustomRunner`.

.. code:: python

  entry_points={
      'ament_lint': [
          'ament_lint_cmake = ament_lint_cmake.main:LintCMakeRunner'
      ]
  },
