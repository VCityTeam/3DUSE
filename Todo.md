 * Reduce number of warnings:
   * [DGTal policy](https://github.com/DGtal-team/DGtal/pull/1182) require the build of a PR to be free of warnings (refer e.g. to the [checklist of this PR](http://stackoverflow.com/questions/15500143/clang-promoting-all-warnings-to-errors-except-for)
   * DGTal achieves by setting the compiler in "warnings as debug" mode when building the PR (and letting the CI trap the error as a classic build fail)
   * Both clang and GCC seem to accept such a compile option (`--Werror`):
     * [clang -Werror](http://stackoverflow.com/questions/15500143/clang-promoting-all-warnings-to-errors-except-for) although not easily documented seems to work
     * Start looking [here whether gcc does the same](http://stackoverflow.com/questions/8466295/gcc-and-clang-warnings-errors-flags)
   * Still for [clang the `-Wno-error=foo`](http://stackoverflow.com/questions/15500143/clang-promoting-all-warnings-to-errors-except-for) turns warning “foo” even if -Werror is specified.   

 * Win32: find a way to expand the PATH variable for the tests to find the dll (filters, citygml...)
   * **What was tried but could not work**: Changing the working directory to become the Bin/Release (or Bin/Debug) i.e where the dll are (with e.g. `set_property( TEST TEST_COMPUTE_ENVELOPE PROPERTY WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE} )` ) won't work. Indeed ctest can only be invoked from directory `Bin`, otherwise it doesn't find the tests themselves. Maybe an alternative would be to the executables of the tests within `Bin` also ?
   * **Plan B: Maybe we can explore** the
     [CTEST_ENVIRONMENT](http://public.kitware.com/pipermail/cmake/2009-September/031899.html) (see [also here](https://cmake.org/Wiki/CMake_Scripting_Of_CTest#More_Settings) and [there](https://cmake.org/pipermail/cmake/2008-February/019989.html)) way of things ?
   * References on the actual plan A (see below):
      * [the central topic](http://www.mail-archive.com/cmake@cmake.org/msg21493.html)
      * [here](https://cmake.org/Bug/view.php?id=15927)
      * [there](https://cmake.org/pipermail/cmake/2012-October/052423.html)
      * [also here](https://cmake.org/pipermail/cmake-developers/2013-June/019217.html)
   * What was tried is to get the following work. Alas nothing works like it should do (only the first entry of the PATH variable is displayed and the trash test fails !)
```cmake
     if( MSVC )
       string( REPLACE ";" "\\;" CURRENT_PATH "$ENV{PATH}")
       set( ENV{PATH} ${CURRENT_PATH} ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE} )
       message( "After substitution of commas" ${CURRENT_PATH} )
     endif()
```
   * In order to debug the above, trial boiled down to the following (which fails poorly):
```cmake
     if( MSVC )
       message( "The coarse result" $ENV{PATH} )
       set( JUNK "$ENV{PATH}")
       string( REPLACE ";" "aaaaa" JUNK "${JUNKO}")
       message("After substitution" ${JUNKO} )
       # Assert the result by promprint the environment from within
	   # cmake. Alas the following fails on Win32 with a message like
	   # "can't find COMMAND" when ${CMAKE_COMMAND} points correctly to cmake.exe !
       message("Cmake binary is here: " ${CMAKE_COMMAND} )
       add_test( NAME trash COMMAND ${CMAKE_COMMAND} -E environment)
     endif()
```

