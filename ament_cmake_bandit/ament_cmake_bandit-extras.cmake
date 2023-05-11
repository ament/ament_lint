find_package(ament_cmake_test QUIET REQUIRED)

include("${ament_cmake_bandit_DIR}/ament_bandit.cmake")

ament_register_extension("ament_lint_auto" "ament_cmake_bandit"
  "ament_cmake_bandit_lint_hook.cmake")
