file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS
  "*.c"
  "*.cc"
  "*.cpp"
  "*.cxx"
  "*.h"
  "*.hh"
  "*.hpp"
  "*.hxx"
)
if(_source_files)
  message(" - Added test 'cppcheck' to perform static code analysis on C / C++ code")
  ament_cppcheck()
endif()
