cmake_minimum_required(VERSION 3.20)
project(ruff_vendor)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

ExternalProject_Add(ruff_external
  GIT_REPOSITORY https://github.com/astral-sh/ruff.git
  GIT_TAG 0.15.15

  CONFIGURE_COMMAND ""
  BUILD_COMMAND cargo build --release
  BUILD_IN_SOURCE TRUE

  INSTALL_COMMAND
    ${CMAKE_COMMAND} -E make_directory
      ${CMAKE_INSTALL_PREFIX}/bin
    COMMAND
      ${CMAKE_COMMAND} -E copy
      <SOURCE_DIR>/target/release/ruff
      ${CMAKE_INSTALL_PREFIX}/bin/ruff
)

install(
  PROGRAMS
    ${CMAKE_INSTALL_PREFIX}/bin/ruff
  DESTINATION bin
)

ament_package()
