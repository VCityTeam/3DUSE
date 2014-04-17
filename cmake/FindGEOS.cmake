#---
# File: FindGEOS.cmake
#
# Find the native GEOS(Geometry Engine - Open Source) includes and libraries.
#
# This module defines:
#
# GEOS_INCLUDE_DIR, where to find geos.h, etc.
# GEOS_LIBRARY, libraries to link against to use GEOS.
# GEOS_FOUND, True if found, false if one of the above are not found.
# 
#  For ossim, typically geos will be system installed which should be found; 
#  or found in the ossim 3rd party dependencies directory from a geos build 
#  and install.  If the latter it will rely on CMAKE_INCLUDE_PATH and 
#  CMAKE_LIBRARY_PATH having the path to the party dependencies directory.
# 
# $Id$
#---

#---
# Find include path:
#---
find_path( GEOS_INCLUDE_DIR geos/version.h
           PATHS 
           /usr/include
           /usr/local/include )

# Find GEOS library:
find_library( GEOS_LIBRARY NAMES geos 
              PATHS 
              /usr/lib64 
              /usr/lib 
              /usr/local/lib )

# Set the GEOS_LIBRARY for MSVC:
IF (MSVC)
	set(GEOS_LIBRARY optimized ${GEOS_LIBRARY} debug ${GEOS_LIBRARY_D})
ENDIF (MSVC)

#---
# This function sets GEOS_FOUND if variables are valid.
#--- 
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args( GEOS DEFAULT_MSG 
                                   GEOS_LIBRARY 
                                   GEOS_INCLUDE_DIR )
