cmake_minimum_required(VERSION 3.20)
project(ruff_vendor)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

# Newest version which supports the oldest Rust version on RHEL (1.92.0)
set(RUFF_VERSION 0.15.11)
ExternalProject_Add(ruff_external
  GIT_REPOSITORY https://github.com/astral-sh/ruff.git
  GIT_TAG ${RUFF_VERSION}
  GIT_SHALLOW TRUE

  CONFIGURE_COMMAND ""

  BUILD_COMMAND
    ${CMAKE_COMMAND} -E env
      CARGO_INCREMENTAL=0
      cargo build --release --verbose -p ruff

  BUILD_IN_SOURCE TRUE

  INSTALL_COMMAND
    ${CMAKE_COMMAND} -E make_directory
      ${CMAKE_INSTALL_PREFIX}/bin
    COMMAND
      ${CMAKE_COMMAND} -E copy
      <SOURCE_DIR>/target/release/ruff${CMAKE_EXECUTABLE_SUFFIX}
      ${CMAKE_INSTALL_PREFIX}/bin/ruff${CMAKE_EXECUTABLE_SUFFIX}
)

ament_package()
