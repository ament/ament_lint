# copied from ament_lint_cmake/ament_lint_cmake-extras.cmake

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

set(ament_lint_cmake_BIN "${ament_lint_cmake_DIR}/../../../bin/ament_lint_cmake")

include("${ament_lint_cmake_DIR}/ament_lint_cmake.cmake")

ament_register_extension("ament_lint_auto" "ament_lint_cmake"
  "ament_lint_cmake_lint_hook.cmake")
