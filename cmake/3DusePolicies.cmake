if (COMMAND cmake_policy)

  # Libraries linked via full path no longer produce linker search paths.
  cmake_policy(SET CMP0003 NEW)

  # ignore CMAKE_SHARED_LIBRARY_<Lang>_FLAGS and honor the
  # POSITION_INDEPENDENT_CODE target property.
  if( POLICY CMP0018 )
    cmake_policy(SET CMP0018 NEW)
  endif ()

  # Automatically link Qt executables to qtmain target on Windows.
  if( POLICY CMP0020 )
    cmake_policy(SET CMP0020 NEW)
  endif ()

  # MACOSX_RPATH is enabled by default.
  if( POLICY CMP0042 )
    cmake_policy(SET CMP0042 NEW)
  endif ()

  # Consume the content of the suffixed COMPILE_DEFINITIONS_<CONFIG> target
  # property when generating the compilation command
  if( POLICY CMP0043 )
    cmake_policy(SET CMP0043 OLD)
  endif ()

  # interpret if() arguments as variables or keywords EVEN when they are
  # quoted or bracketed.
  if(POLICY CMP0054)
    cmake_policy(SET CMP0054 OLD)
  endif ()

endif ()
