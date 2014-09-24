cmake_minimum_required(VERSION 2.8.3)

project(ament_uncrustify NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_test REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "ament_uncrustify-extras.cmake"
)

install(
  PROGRAMS bin/ament_uncrustify
  DESTINATION bin
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES ament_code_style.cfg
  DESTINATION share/${PROJECT_NAME}
)
