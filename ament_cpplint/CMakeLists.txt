cmake_minimum_required(VERSION 2.8.3)

project(ament_cpplint NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_test REQUIRED)

ament_python_install_package(${PROJECT_NAME})

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

if(AMENT_ENABLE_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  set(python_files "ament_cpplint/__init__.py" "bin/ament_cpplint")

  find_package(ament_cmake_pep8 REQUIRED)
  ament_pep8(${python_files})

  find_package(ament_pyflakes REQUIRED)
  ament_pyflakes(${python_files})
endif()
