ament_lint_auto_py
===============

.. TODO
The package simplifies using multiple linters as part of pytest tests.
It reduces the amount of Python code to a bare minimum.

``test_ament_lint_auto`:

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

.. TODO
How to exclude linter modules with ament_lint_auto?
---------------------------------------------------

Linter modules can be excluded via the CMake list variable `AMENT_LINT_AUTO_EXCLUDE`.

As an example to exclude the `copyright` linter:

``CMakeLists.txt``:

.. code:: cmake

    # this must happen before the invocation of ament_package()
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
      ament_lint_auto_find_test_dependencies()
    endif()


How to exclude files with ament_lint_auto?
------------------------------------------

Linter hooks shall conform to the ament_lint_auto convention of excluding files
specified in the CMake list variable `AMENT_LINT_AUTO_FILE_EXCLUDE`.
As such, the CMake snippet from above can be modified to exclude files across
all linters with one addition.

``CMakeLists.txt``:

.. code:: cmake

    # this must happen before the invocation of ament_package()
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      set(AMENT_LINT_AUTO_FILE_EXCLUDE /path/to/ignored_file ...)
      ament_lint_auto_find_test_dependencies()
    endif()

For a more specific example, this excludes all python files matching a pattern using globbing.
Multiple expressions can be combined on multiple lines.
It might be a good idea to issue a warning to developers that linting is disabled
if you plan to enable it at some point.

.. code:: cmake

      file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
        # Exclude all the python files in src directory
        src/*.py
        # Exclude all the c++ implementation files in test directory
        test/*.cpp
      )
      message(AUTHOR_WARNING
          "Ament lint auto tests are disabled on the following: "
          ${AMENT_LINT_AUTO_FILE_EXCLUDE}
      )

