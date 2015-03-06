# copied from ament_clang_format/ament_clang_format-extras.cmake

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

set(ament_clang_format_BIN "${ament_clang_format_DIR}/../../../bin/ament_clang_format")

include("${ament_clang_format_DIR}/ament_clang_format.cmake")

ament_register_extension("ament_lint_auto" "ament_clang_format"
  "ament_clang_format_lint_hook.cmake")
