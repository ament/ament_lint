cmake_minimum_required(VERSION 2.8.3)

project(ament_lint_cmake NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_test REQUIRED)

# include CMake functions of this package
set(ament_lint_cmake_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("ament_lint_cmake-extras.cmake" NO_POLICY_SCOPE)
set(ament_lint_cmake_BIN "${CMAKE_CURRENT_SOURCE_DIR}/bin/ament_lint_cmake")

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "ament_lint_cmake-extras.cmake"
)

install(
  PROGRAMS bin/ament_lint_cmake
  DESTINATION bin
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(AMENT_ENABLE_TESTING)
  ament_lint_cmake(lint_cmake)
endif()
