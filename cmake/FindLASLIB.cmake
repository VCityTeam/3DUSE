#---
# File: FindLasLib.cmake
#
# Find the native LasLib includes and libraries.
# Martial Tola - October 2014
#
# This module defines:
#
# LASLIB_INCLUDE_DIR, where to find lasreader.hpp, etc.
# LASLIB_LIBRARY, libraries to link against to use LASLIB.
# LASLIB_FOUND, True if found, false if one of the above are not found.
#---

#---
# Find include path:
#---
find_path( LASLIB_INCLUDE_DIR lasreader.hpp
           PATHS 
           /usr/include
           /usr/local/include )

# Find LASLIB library:
find_library( LASLIB_LIBRARY NAMES laslib
              PATHS 
              /usr/lib64 
              /usr/lib 
              /usr/local/lib )

# Set the LASLIB_LIBRARY for MSVC:
IF (MSVC)
	set(LASLIB_LIBRARY optimized ${LASLIB_LIBRARY} debug ${LASLIB_LIBRARY_D})
ENDIF (MSVC)

#---
# This function sets LASLIB_FOUND if variables are valid.
#--- 
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args( LASLIB DEFAULT_MSG 
                                   LASLIB_LIBRARY 
                                   LASLIB_INCLUDE_DIR )
