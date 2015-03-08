file(GLOB_RECURSE _cmake_files FOLLOW_SYMLINKS
  "CMakeLists.txt"
  "*.cmake"
)
if(_cmake_files)
  message(" - Added test 'lint_cmake' to check CMake code style")
  ament_lint_cmake()
endif()
