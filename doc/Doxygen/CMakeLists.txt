find_package( Doxygen )

if( NOT DOXYGEN_FOUND)
  return()
endif()

if( DOXYGEN_DOT_FOUND )
  set( DOXYGEN_HAVE_DOT "YES" )
else()
  set( DOXYGEN_HAVE_DOT "NO" )
endif()

# we run several doxygen builds in parallel, so we reduce the number of procs
# used by each doxygen to run dot
if(NOT CMAKE_VERSION VERSION_LESS 2.8.5)
  include(ProcessorCount)
  processorcount(proc_count)
  math(EXPR DOT_JOBS "${proc_count} / 4")
  if(DOT_JOBS LESS 1)
    set(DOT_JOBS 1)
  endif()
else()
  set(DOT_JOBS 2)
endif()

set( DOXYGEN_FILE_PATTERNS "*.h *.hpp *.cpp *.cxx *.dox *.md")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
  ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
  @ONLY)

# Construct the list of source dependencies
set( DEPENDENCIES )
file( GLOB_RECURSE files ${DOXYGEN_FILE_PATTERNS} )
list( APPEND DEPENDENCIES ${files} )

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/html/index.html
  COMMAND ${DOXYGEN} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
  COMMENT "Generating Doxygen html output..."
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile ${DEPENDENCIES}
)

add_custom_target(
  DocDoxygen ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/index.html
)
