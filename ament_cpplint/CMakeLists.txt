cmake_minimum_required(VERSION 2.8.3)

project(ament_cpplint)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_index REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_test REQUIRED)

ament_package(
  CONFIG_EXTRAS "ament_cpplint-extras.cmake"
)

install(
  PROGRAMS bin/ament_cpplint
  DESTINATION bin
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})
