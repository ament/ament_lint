cmake_minimum_required(VERSION 3.20)
project(ruff_vendor)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

# Newest version which supports the oldest Rust version on RHEL (1.92.0)
set(RUFF_VERSION 0.15.11)

option(RUFF_USE_PREBUILT
  "Download a prebuilt Ruff release instead of building from source"
  OFF
)

if(RUFF_USE_PREBUILT)
  if(WIN32)
    set(RUFF_ARCHIVE
      "ruff-x86_64-pc-windows-msvc.zip")
  elseif(UNIX)
    set(RUFF_ARCHIVE
      "ruff-x86_64-unknown-linux-gnu.tar.gz")
  else()
    message(FATAL_ERROR "Unsupported platform")
  endif()

  ExternalProject_Add(ruff_external
    URL
      https://github.com/astral-sh/ruff/releases/download/${RUFF_VERSION}/${RUFF_ARCHIVE}

    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""

    INSTALL_COMMAND
      ${CMAKE_COMMAND} -E make_directory
        ${CMAKE_INSTALL_PREFIX}/bin
      COMMAND
        ${CMAKE_COMMAND} -E tar xzf <DOWNLOADED_FILE>
      COMMAND
        ${CMAKE_COMMAND} -E copy
        <SOURCE_DIR>/ruff${CMAKE_EXECUTABLE_SUFFIX}
        ${CMAKE_INSTALL_PREFIX}/bin/ruff${CMAKE_EXECUTABLE_SUFFIX}

    LOG_DOWNLOAD ON
    LOG_INSTALL ON
    LOG_OUTPUT_ON_FAILURE ON
  )

else()

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

    LOG_DOWNLOAD ON
    LOG_BUILD ON
    LOG_INSTALL ON
    LOG_OUTPUT_ON_FAILURE ON
  )
endif()

ament_package()
