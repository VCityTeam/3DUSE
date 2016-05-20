project(blah)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX /tmp/junk CACHE PATH "a" FORCE )
endif()

cmake_minimum_required(VERSION 3.0)

#set(CMAKE_MACOSX_RPATH ON)

add_subdirectory(subdir)
add_executable(mytest test.c)
add_library(mylib SHARED mylib.c)
target_link_libraries(mylib mylib2)
target_link_libraries(mytest mylib)

install(TARGETS mylib
  LIBRARY DESTINATION .
  BUNDLE DESTINATION .)
install(TARGETS mytest RUNTIME DESTINATION .)

# https://cmake.org/Bug/view.php?id=14444
# Le make install de mylib2 n'est pas fait car l'ordre d'execution fait que
# le fixup_bundle est effectué avant et du coup cela plante.
# Can not be fixed with add_dependencies() because of
#            https://cmake.org/Bug/view.php?id=8438

set(APPS    mytest)
set(PLUGINS "")
set(DIRS    "${CMAKE_BINARY_DIR}/subdir" "${CMAKE_BINARY_DIR}")

if( true )
  install( CODE "
    include(BundleUtilities)
    fixup_bundle(\"${APPS}\" \"\${PLUGINS}\" \"${DIRS}\")
    " COMPONENT Runtime
   )
endif()
