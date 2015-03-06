# copied from ament_uncrustify/ament_uncrustify-extras.cmake

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

set(ament_uncrustify_BIN "${ament_uncrustify_DIR}/../../../bin/ament_uncrustify")
set(ament_uncrustify_CFG "${ament_uncrustify_DIR}/../ament_code_style.cfg")

include("${ament_uncrustify_DIR}/ament_uncrustify.cmake")

ament_register_extension("ament_lint_auto" "ament_uncrustify"
  "ament_uncrustify_lint_hook.cmake")
