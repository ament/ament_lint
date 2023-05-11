find_package(ament_cmake_core REQUIRED)

file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS
  "*.py"
)
if(_source_files)
  message(STATUS "Added test 'bandit' to perform static code analysis on Python code")

  # Get exclude paths for added targets
  set(_all_exclude "")
  if(DEFINED ament_cmake_bandit_ADDITIONAL_EXCLUDE)
    list(APPEND _all_exclude ${ament_cmake_bandit_ADDITIONAL_EXCLUDE})
  endif()

  if(DEFINED AMENT_LINT_AUTO_FILE_EXCLUDE)
    list(APPEND _all_exclude ${AMENT_LINT_AUTO_FILE_EXCLUDE})
  endif()

  message(
    STATUS "Configured bandit exclude dirs and/or files: ${_all_exclude}"
  )
  ament_bandit(
    EXCLUDE ${_all_exclude}
  )
endif()
