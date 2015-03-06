file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS
  "*.cmake"

  "*.c"
  "*.cc"
  "*.cpp"
  "*.cxx"
  "*.h"
  "*.hh"
  "*.hpp"
  "*.hxx"

  "*.py"
)
if(_source_files)
  message(" - Added test 'copyright' to check for copyright in CMake / C / C++ / Python code")
  ament_copyright(copyright)
endif()
