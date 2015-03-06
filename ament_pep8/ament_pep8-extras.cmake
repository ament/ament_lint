# copied from ament_pep8/ament_pep8-extras.cmake

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

set(ament_pep8_BIN "${ament_pep8_DIR}/../../../bin/ament_pep8")

include("${ament_pep8_DIR}/ament_pep8.cmake")

ament_register_extension("ament_lint_auto" "ament_pep8"
  "ament_pep8_lint_hook.cmake")
