# - Provide access to the OpenSceneGraph runtime files for bundling in
# an installation or package.
#
# Sets these variables:
#  - OSGDB_PLUGINS_RELEASE
#  - OSGDB_PLUGINS_DEBUG
#  - OSGWRAPPER_PLUGINS_RELEASE
#  - OSGWRAPPER_PLUGINS_DEBUG
#  - OSG_RUNTIME_LIBRARY_DIR
#  - OSG_PATH_TO_PLUGINS
#
# Creates this function:
#  - install_osg_plugins( {varNameForOutputFilenames} )
#
# Requires these CMake modules:
#  no additional modules required
#
# Original Author:
# 2009-2010 Ryan Pavlik <rpavlik@iastate.edu> <abiryan@ryand.net>
# http://academic.cleardefinition.com
# Iowa State University HCI Graduate Program/VRAC
#
# Copyright Iowa State University 2009-2010.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)
#
#
# 2014 -> MT changes

function(_osgbundle_split_debug_versions releasevar debugvar)
	set(release)
	set(debug)
	foreach(fn ${ARGN})
		get_filename_component(name "${fn}" NAME_WE)
		if("${name}" MATCHES "d$")
			list(APPEND debug "${fn}")
		else()
			list(APPEND release "${fn}")
		endif()
	endforeach()
	set(${releasevar} ${release} PARENT_SCOPE)
	set(${debugvar} ${debug} PARENT_SCOPE)
endfunction()

function(_osgbundle_find_plugins varprefix filenameprefix)
	file(GLOB
		all
		"${OSG_RUNTIME_LIBRARY_DIR}/osgPlugins-${OPENSCENEGRAPH_VERSION}/${filenameprefix}*${CMAKE_SHARED_LIBRARY_SUFFIX}")
	_osgbundle_split_debug_versions(${varprefix}_PLUGINS_RELEASE
		${varprefix}_PLUGINS_DEBUG
		${all})
	set(${varprefix}_PLUGINS_RELEASE
		"${${varprefix}_PLUGINS_RELEASE}"
		PARENT_SCOPE)
	set(${varprefix}_PLUGINS_DEBUG
		"${${varprefix}_PLUGINS_DEBUG}"
		PARENT_SCOPE)
endfunction()

if(OPENSCENEGRAPH_FOUND)
	if(WIN32)
		get_filename_component(_osglibdir "${OSG_LIBRARY}" PATH)
		get_filename_component(_osgroot "${_osglibdir}/.." ABSOLUTE)
		set(OSG_RUNTIME_LIBRARY_DIR "${_osgroot}/bin")
		#set(OSG_PATH_TO_PLUGINS "bin/osgPlugins-${OPENSCENEGRAPH_VERSION}/")
		set(OSG_PATH_TO_PLUGINS "osgPlugins-${OPENSCENEGRAPH_VERSION}")
	else()
		get_filename_component(_osglibdir "${OSG_LIBRARY}" PATH)
		set(OSG_RUNTIME_LIBRARY_DIR "${_osglibdir}")
		#set(OSG_PATH_TO_PLUGINS "lib/osgPlugins-${OPENSCENEGRAPH_VERSION}/")
		set(OSG_PATH_TO_PLUGINS "osgPlugins-${OPENSCENEGRAPH_VERSION}")
		IF(APPLE)
			SET(OSG_PATH_TO_PLUGINS "${PRJ_NAME}.app/Contents/MacOS/${OSG_PATH_TO_PLUGINS}") # MT
		ENDIF(APPLE)
	endif()

	# Find the osgDB plugins
	#_osgbundle_find_plugins(OSGDB osgdb) # MT
	#_osgbundle_find_plugins(OSGWRAPPER osgwrapper) # MT

	# MT
	_osgbundle_find_plugins(OSGDB_freetype osgdb_freetype)
	_osgbundle_find_plugins(OSGDB_bmp osgdb_bmp)
	_osgbundle_find_plugins(OSGDB_gif osgdb_gif)
	_osgbundle_find_plugins(OSGDB_jpeg osgdb_jpeg)
	_osgbundle_find_plugins(OSGDB_png osgdb_png)
	_osgbundle_find_plugins(OSGDB_pnm osgdb_pnm)
	_osgbundle_find_plugins(OSGDB_tga osgdb_tga)
	_osgbundle_find_plugins(OSGDB_tiff osgdb_tiff)
	# MT
endif()

function(install_osg_plugins var)
	set(INSTALLEDPLUGINS)
	#foreach(plugin ${OSGDB_PLUGINS_RELEASE} ${OSGWRAPPER_PLUGINS_RELEASE}) # MT
	foreach(plugin ${OSGDB_freetype_PLUGINS_RELEASE} ${OSGDB_bmp_PLUGINS_RELEASE} ${OSGDB_gif_PLUGINS_RELEASE} ${OSGDB_jpeg_PLUGINS_RELEASE}
		${OSGDB_png_PLUGINS_RELEASE} ${OSGDB_pnm_PLUGINS_RELEASE} ${OSGDB_tga_PLUGINS_RELEASE} ${OSGDB_tiff_PLUGINS_RELEASE})
		install(FILES "${plugin}"
			DESTINATION "${OSG_PATH_TO_PLUGINS}"
			CONFIGURATIONS Release RelWithDebInfo MinSizeRel # MT : important, under Linux and Mac OS X need "cmake .. -DCMAKE_BUILD_TYPE=Release" for example
			COMPONENT ${PRJ_NAME} # MT
			)
		get_filename_component(name "${plugin}" NAME)
		list(APPEND INSTALLEDPLUGINS "${OSG_PATH_TO_PLUGINS}/${name}")
	endforeach()
	#foreach(plugin ${OSGDB_PLUGINS_DEBUG} ${OSGWRAPPER_PLUGINS_DEBUG}) # MT
	foreach(plugin ${OSGDB_freetype_PLUGINS_DEBUG} ${OSGDB_bmp_PLUGINS_DEBUG} ${OSGDB_gif_PLUGINS_DEBUG} ${OSGDB_jpeg_PLUGINS_DEBUG}
		${OSGDB_png_PLUGINS_DEBUG} ${OSGDB_pnm_PLUGINS_DEBUG} ${OSGDB_tga_PLUGINS_DEBUG} ${OSGDB_tiff_PLUGINS_DEBUG})
		install(FILES "${plugin}"
			DESTINATION "${OSG_PATH_TO_PLUGINS}"
			CONFIGURATIONS Debug # MT : important, under Linux and Mac OS X need "cmake .. -DCMAKE_BUILD_TYPE=Debug" for example
			COMPONENT ${PRJ_NAME} # MT
			)
		get_filename_component(name "${plugin}" NAME) # MT
		list(APPEND INSTALLEDPLUGINS "${OSG_PATH_TO_PLUGINS}/${name}") # MT
	endforeach()
	set(${var} ${INSTALLEDPLUGINS} PARENT_SCOPE)
endfunction()
