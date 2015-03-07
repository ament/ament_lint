cmake_minimum_required(VERSION 2.8.3)

project(ament_copyright NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_test REQUIRED)

# include CMake functions of this package
set(ament_copyright_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("ament_copyright-extras.cmake" NO_POLICY_SCOPE)
set(ament_copyright_BIN "${CMAKE_CURRENT_SOURCE_DIR}/bin/ament_copyright")

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "ament_copyright-extras.cmake"
)

install(
  PROGRAMS bin/ament_copyright
  DESTINATION bin
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(AMENT_ENABLE_TESTING)
  set(python_files "ament_copyright" "bin/ament_copyright")

  find_package(ament_pep8 REQUIRED)
  ament_pep8(pep8 ${python_files})

  find_package(ament_pyflakes REQUIRED)
  ament_pyflakes(pyflakes ${python_files})

  find_package(ament_lint_cmake REQUIRED)
  ament_lint_cmake(lint_cmake)

  ament_copyright(copyright "ament_copyright" "bin/ament_copyright")
endif()
