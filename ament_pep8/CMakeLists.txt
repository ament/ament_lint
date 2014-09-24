cmake_minimum_required(VERSION 2.8.3)

project(ament_pep8)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_test REQUIRED)

# include CMake functions of this package
set(ament_pep8_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("ament_pep8-extras.cmake" NO_POLICY_SCOPE)
set(ament_pep8_BIN "${CMAKE_CURRENT_SOURCE_DIR}/bin/ament_pep8")

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "ament_pep8-extras.cmake"
)

install(
  PROGRAMS bin/ament_pep8
  DESTINATION bin
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(AMENT_ENABLE_TESTING)
  ament_pep8(pep8 "ament_pep8" "bin/ament_pep8")
endif()
