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
  message(" - Added test 'cpplint' to check C / C++ code against the Google style")
  ament_cpplint()
endif()
