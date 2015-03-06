#
# Invoke find_package() for all test dependencies.
#
# All found package names are appended to the
# ``${PROJECT_NAME}_FOUND_TEST_DEPENDS`` variables.
#
# @public
#
macro(ament_lint_auto_find_test_dependencies)
  if(ARGN)
    message(FATAL_ERROR "ament_lint_auto_find_test_dependencies() called with "
      "unused arguments: ${ARGN}")
  endif()

  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  # try to find_package() all test dependencies
  foreach(_dep ${${PROJECT_NAME}_TEST_DEPENDS})
    find_package(${_dep} QUIET)
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_TEST_DEPENDS ${_dep})
    endif()
  endforeach()
endmacro()
