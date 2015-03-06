# copied from ament_copyright/ament_copyright-extras.cmake

find_package(ament_cmake_test REQUIRED)

set(ament_copyright_BIN "${ament_copyright_DIR}/../../../bin/ament_copyright")

include("${ament_copyright_DIR}/ament_copyright.cmake")

ament_register_extension("ament_lint_auto" "ament_copyright"
  "ament_copyright_lint_hook.cmake")
