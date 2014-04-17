#---
# File: FindGEOS.cmake
#
# Find the native GEOS(Geometry Engine - Open Source) includes and libraries.
#
# This module defines:
#
# GEOS_INCLUDE_DIR, where to find geos.h, etc.
# GEOS_LIBRARY, libraries to link against to use GEOS.  Currently there are
# two looked for, geos and geos_c libraries.
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
# Note: Version < 3.3.0 do not have geos.h in the geos sub directory; hence,
# the check for both "geos/geos.h" and "geos.h".
#---
find_path( GEOS_INCLUDE_DIR geos/geos.h geos.h
           PATHS 
           /usr/include
           /usr/local/include )

# Find GEOS library:
find_library( GEOS_LIB NAMES geos 
              PATHS 
              /usr/lib64 
              /usr/lib 
              /usr/local/lib )

# Find GEOS C library:
find_library( GEOS_C_LIB NAMES geos_c 
              PATHS 
              /usr/lib64 
              /usr/lib 
              /usr/local/lib )

# Set the GEOS_LIBRARY:
if( GEOS_LIB AND GEOS_C_LIB )
   set( GEOS_LIBRARY ${GEOS_LIB} ${GEOS_C_LIB} CACHE STRING INTERNAL )
endif(GEOS_LIB AND GEOS_C_LIB )

#---
# This function sets GEOS_FOUND if variables are valid.
#--- 
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args( GEOS DEFAULT_MSG 
                                   GEOS_LIBRARY 
                                   GEOS_INCLUDE_DIR )

if( GEOS_FOUND )
   if( NOT GEOS_FIND_QUIETLY )
      message( STATUS "Found GEOS..." )
   endif( NOT GEOS_FIND_QUIETLY )
else( GEOS_FOUND )
   if( NOT GEOS_FIND_QUIETLY )
      message( WARNING "Could not find GEOS" )
   endif( NOT GEOS_FIND_QUIETLY )
endif( GEOS_FOUND )

if( NOT GEOS_FIND_QUIETLY )
   message( STATUS "GEOS_INCLUDE_DIR=${GEOS_INCLUDE_DIR}" )
   message( STATUS "GEOS_LIBRARY=${GEOS_LIBRARY}" )
endif( NOT GEOS_FIND_QUIETLY )
