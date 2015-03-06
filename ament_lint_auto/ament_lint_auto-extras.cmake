# copied from ament_lint_auto/ament_lint_auto-extras.cmake

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

include(
  "${ament_lint_auto_DIR}/ament_lint_auto_find_test_dependencies.cmake")

ament_register_extension("ament_package" "ament_lint_auto"
  "ament_lint_auto_package_hook.cmake")
