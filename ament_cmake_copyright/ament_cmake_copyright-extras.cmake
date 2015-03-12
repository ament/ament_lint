# copied from ament_cmake_copyright/ament_cmake_copyright-extras.cmake

find_package(ament_cmake_test REQUIRED)

include("${ament_cmake_copyright_DIR}/ament_copyright.cmake")

ament_register_extension("ament_lint_auto" "ament_cmake_copyright"
  "ament_cmake_copyright_lint_hook.cmake")
