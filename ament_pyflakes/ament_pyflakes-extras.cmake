# copied from ament_pyflakes/ament_pyflakes-extras.cmake

find_package(ament_cmake_test REQUIRED)

set(ament_pyflakes_BIN "${ament_pyflakes_DIR}/../../../bin/ament_pyflakes")

include("${ament_pyflakes_DIR}/ament_pyflakes.cmake")

ament_register_extension("ament_lint_auto" "ament_pyflakes"
  "ament_pyflakes_lint_hook.cmake")
