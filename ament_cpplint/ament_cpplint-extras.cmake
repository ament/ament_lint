# copied from ament_cpplint/ament_cpplint-extras.cmake

find_package(ament_cmake_test REQUIRED)

set(ament_cpplint_BIN "${ament_cpplint_DIR}/../../../bin/ament_cpplint")

include("${ament_cpplint_DIR}/ament_cpplint.cmake")

ament_register_extension("ament_lint_auto" "ament_cpplint"
  "ament_cpplint_lint_hook.cmake")
