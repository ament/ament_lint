cmake_minimum_required(VERSION 2.8.3)

project(ament_pyflakes)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_environment REQUIRED)
find_package(ament_cmake_index REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_test REQUIRED)

# include CMake functions of this package
set(ament_pyflakes_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("ament_pyflakes-extras.cmake" NO_POLICY_SCOPE)
set(ament_pyflakes_BIN "${CMAKE_CURRENT_SOURCE_DIR}/bin/ament_pyflakes")

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "ament_pyflakes-extras.cmake"
)

install(
  PROGRAMS bin/ament_pyflakes
  DESTINATION bin
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(AMENT_ENABLE_TESTING)
  ament_pyflakes(pyflakes "ament_pyflakes" "bin/ament_pyflakes")
endif()
