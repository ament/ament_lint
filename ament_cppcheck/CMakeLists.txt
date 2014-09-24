cmake_minimum_required(VERSION 2.8.3)

project(ament_cppcheck)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_test REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "ament_cppcheck-extras.cmake"
)

install(
  PROGRAMS bin/ament_cppcheck
  DESTINATION bin
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
