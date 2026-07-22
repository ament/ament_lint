cmake_minimum_required(VERSION 3.20)
project(ruff_vendor)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

ExternalProject_Add(ruff_external
  GIT_REPOSITORY https://github.com/astral-sh/ruff.git
  GIT_TAG 0.15.15
  GIT_SHALLOW TRUE

  CONFIGURE_COMMAND ""
  BUILD_COMMAND cargo build --release --verbose
  BUILD_IN_SOURCE TRUE

  INSTALL_COMMAND
  ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_INSTALL_PREFIX}/bin
  COMMAND
    ${CMAKE_COMMAND} -E copy
    <SOURCE_DIR>/target/release/ruff${CMAKE_EXECUTABLE_SUFFIX}
    ${CMAKE_INSTALL_PREFIX}/bin/ruff${CMAKE_EXECUTABLE_SUFFIX}

  LOG_DOWNLOAD ON
  LOG_BUILD ON
  LOG_INSTALL ON
  LOG_OUTPUT_ON_FAILURE ON
)

ament_package()
