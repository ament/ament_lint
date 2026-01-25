ament_lint_auto_py
===============

The package simplifies using multiple linters as part of pytest tests.
It reduces the amount of Python code to a bare minimum.

``test_ament_lint_auto.py`:

.. code:: python

  import pytest


  @pytest.mark.ament_lint_auto_py
  @pytest.mark.linter
  def test_ament_lint_auto(run_entry_point) -> None:
      """Run all installed linter dynamically."""
      rc = run_entry_point()
      assert rc == 0, f'Linter[{run_entry_point.NAME}] failed with exit code {rc}'

The set of linters to be used is then only specified in the package manifest as
test dependencies.

``package.xml``:

.. code:: xml

    <test_depend>ament_lint_auto_py</test_depend>

    <!-- add test dependencies on any linter, e.g. -->
    <test_depend>ament_clang_format</test_depend>
    <test_depend>ament_cppcheck</test_depend>
    <test_depend>ament_pycodestyle</test_depend>

Since recursive dependencies are also being used a single test dependency is
sufficient to test with a set of common linters.

``package.xml``:

.. code:: xml

    <test_depend>ament_lint_auto_py</test_depend>

    <!-- this recursively depends on a set of common linters -->
    <test_depend>ament_lint_common_py</test_depend>


How to exclude linter modules with ament_lint_auto?
---------------------------------------------------

Linter modules can be excluded via the environment variable `AMENT_LINT_AUTO_EXCLUDE`.

As an example to exclude the `copyright` linter:

.. code:: bash
    export AMENT_LINT_EXCLUDE="ament_copyright;"


How to exclude files with ament_lint_auto?
------------------------------------------

Linter hooks shall conform to the ament_lint_auto convention of excluding files
specified in the environment list variable `AMENT_LINT_AUTO_FILE_EXCLUDE`.

.. code:: bash
    export AMENT_LINT_EXCLUDE="/path/to/ignored_file"

For a more specific example, this excludes all python files matching a pattern using globbing.
Multiple expressions can be combined on multiple lines.

.. code:: bash
  .. code:: bash
    export AMENT_LINT_EXCLUDE="src/*.py;test/*.cpp"
