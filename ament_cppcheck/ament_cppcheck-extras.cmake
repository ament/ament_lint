# copied from ament_cppcheck/ament_cppcheck-extras.cmake

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

set(ament_cppcheck_BIN "${ament_cppcheck_DIR}/../../../bin/ament_cppcheck")

include("${ament_cppcheck_DIR}/ament_cppcheck.cmake")

ament_register_extension("ament_lint_auto" "ament_cppcheck"
  "ament_cppcheck_lint_hook.cmake")
