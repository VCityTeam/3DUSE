if (WITH_QT5)
  cmake_minimum_required(VERSION 2.8.9) # because 2.8.8 has a bug with QT5_USE_MODULES
else(WITH_QT5)
  cmake_minimum_required(VERSION 2.8)
endif(WITH_QT5)

if (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  IF(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)
  ENDIF(POLICY CMP0020)
  if(POLICY CMP0054)
	cmake_policy(SET CMP0054 OLD)
  endif(POLICY CMP0054)
endif (COMMAND cmake_policy)

# Project configuration
set(PRJ_NAME 3DUSE)

SET(PRJ_VERSION_MAJOR 0)
SET(PRJ_VERSION_MINOR 3)
SET(PRJ_VERSION_PATCH 2)
SET(PRJ_VERSION "${PRJ_VERSION_MAJOR}.${PRJ_VERSION_MINOR}.${PRJ_VERSION_PATCH}")
message(STATUS "${PRJ_NAME} ${PRJ_VERSION}")

project( ${PRJ_NAME} )
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/${PRJ_NAME}_deploy_${PRJ_VERSION}")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# set Release as default build target
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: Debug, Release."
      FORCE)
endif ()
message("Build type : ${CMAKE_BUILD_TYPE}")

#------------------------------------------------------------------------------
# Specific compiler options
#------------------------------------------------------------------------------
# GNUCXX
#------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX)
  SET(ENABLE_CXX11 "-std=c++11")

  EXECUTE_PROCESS(COMMAND "${CMAKE_CXX_COMPILER} -dumpversion" OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION LESS 4.7)
    SET(ENABLE_CXX11 "-std=c++0x")
  endif()

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ENABLE_CXX11}")
  message(STATUS "GNUCXX compiler detected (with ${ENABLE_CXX11} activated)")
endif()
#------------------------------------------------------------------------------
# Clang (export CXX=clang++ CC=clang)
#------------------------------------------------------------------------------
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  SET(ENABLE_CXX11 "-std=c++11")

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ENABLE_CXX11} -Qunused-arguments")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ENABLE_CXX11} -Qunused-arguments")
  message(STATUS "Clang compiler detected (with ${ENABLE_CXX11} activated)")
  if ( ${CMAKE_BUILD_TYPE} MATCHES "Debug" )
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdocumentation")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -g -fsanitize=address -fno-omit-frame-pointer")
#    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O1 -g -fsanitize=address -fno-omit-frame-pointer")
  endif( ${CMAKE_BUILD_TYPE} MATCHES "Debug" )
endif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
#------------------------------------------------------------------------------
# MSVC
#------------------------------------------------------------------------------
# /MP for multiple compilation units (cl.exe) ; without any value, it is set automatically depending on your number of main threads
# If you are compiling on Windows, the preprocessor symbols 'min' or 'max' are defined. Do #define NOMINMAX to prevent windows.h from defining these symbols.
#
# /FORCE:MULTIPLE for osgDB ifstream / ofstream conflict
if(MSVC)
	SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /D \"NOMINMAX\"")

	SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:MULTIPLE")
endif()
#------------------------------------------------------------------------------

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# WIN32 x64 VCity binary kit
IF(WIN32)
	if(DEFINED ENV{VCITY_KIT_ROOT})
		set( VCITY_KIT_ROOT $ENV{VCITY_KIT_ROOT} )
	endif()
	if( NOT DEFINED VCITY_KIT_ROOT )
		message(FATAL_ERROR "VCITY_KIT_ROOT not set.  Please set VCITY_KIT_ROOT.")
	endif( NOT DEFINED VCITY_KIT_ROOT )

	if(WITH_QT5)
		# with qt5
		set(QT5_DIR				${VCITY_KIT_ROOT}/Qt/Qt5.1.0/5.1.0/msvc2012_64_opengl)
	else(WITH_QT5)
		# with qt4
		set(QTDIR				${VCITY_KIT_ROOT}/Qt/qt-4.8.5-x64-msvc2012)
	endif(WITH_QT5)

	set(ASSIMP_ROOT_DIR			${VCITY_KIT_ROOT}/assimp-3.0.1270 )

	set(OSG_DIR					${VCITY_KIT_ROOT}/osg/osg-3.2.1-bin-without-qt)

	set(GDAL_INCLUDE_DIR		${OSG_DIR}/../3rdParty/x64/include)
	set(GDAL_LIBRARY			${OSG_DIR}/../3rdParty/x64/lib/gdal_i.lib)

	set(LIBXML2_INCLUDE_DIR		${OSG_DIR}/../3rdParty/x64/include/libxml2)
	set(LIBXML2_LIBRARIES		${OSG_DIR}/../3rdParty/x64/lib/libxml2.lib) #libxml2D.lib

	#set(PROJ_INCLUDE_DIR		${OSG_DIR}/../3rdParty/x64/include)
	#set(PROJ_LIBRARY			${OSG_DIR}/../3rdParty/x64/lib/proj.lib)

	#set(GEOS_INCLUDE_DIR		${VCITY_KIT_ROOT}/geos-3.4.2/include)
	#set(GEOS_LIBRARY			${VCITY_KIT_ROOT}/geos-3.4.2/lib/Release/libgeos.lib)
	#set(GEOS_LIBRARY_D			${VCITY_KIT_ROOT}/geos-3.4.2/lib/Debug/libgeos.lib)

	set(LASLIB_INCLUDE_DIR		${VCITY_KIT_ROOT}/laslib/include)
	set(LASLIB_LIBRARY			${VCITY_KIT_ROOT}/laslib/lib/Release/laslib.lib)
	set(LASLIB_LIBRARY_D		${VCITY_KIT_ROOT}/laslib/lib/Debug/laslib.lib)
ENDIF(WIN32)

# Find Qt
if (WITH_QT5)
  # QT5 Handling
  if(DEFINED ENV{QT5_DIR})
    set( QT5_DIR $ENV{QT5_DIR} )
  endif()
  SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${QT5_DIR})

  # Instruct CMake to run moc automatically when needed
  set(CMAKE_AUTOMOC ON)

  FIND_PACKAGE(Qt5Core)
  if(${Qt5Core_FOUND})
    message(STATUS "--> Qt5Core found.")
    FIND_PACKAGE(Qt5Widgets)
    if(${Qt5Widgets_FOUND})
      message(STATUS "--> Qt5Widgets found.")
      FIND_PACKAGE(Qt5Xml)
      if(${Qt5Xml_FOUND})
	message(STATUS "--> Qt5Xml found.")
	FIND_PACKAGE(Qt5OpenGL)
	if(${Qt5OpenGL_FOUND})
	  message(STATUS "--> Qt5OpenGL found.")
	  if (WITH_QT3D)
	    FIND_PACKAGE(Qt53D)
	    if(${Qt53D_FOUND})
	      message(STATUS "--> Qt53D found.")
	    else(${Qt53D_FOUND})
	      message(FATAL_ERROR "Qt53D not found.")
	    endif(${Qt53D_FOUND})
	  endif(WITH_QT3D)
	else(${Qt5OpenGL_FOUND})
	  message(FATAL_ERROR "Qt5OpenGL not found.")
	endif(${Qt5OpenGL_FOUND})
      else(${Qt5Xml_FOUND})
	message(FATAL_ERROR "Qt5Xml not found.")
      endif(${Qt5Xml_FOUND})
    else(${Qt5Widgets_FOUND})
      message(FATAL_ERROR "Qt5Widgets not found.")
    endif(${Qt5Widgets_FOUND})
  else(${Qt5Core_FOUND})
    message(FATAL_ERROR "Qt5Core not found. Please set QT5_DIR.")
  endif(${Qt5Core_FOUND})
else(WITH_QT5)
  # QT4 Handling
  if (QTDIR)
    set( ENV{QTDIR} ${QTDIR} )
  endif()
  set( QT_USE_QTMAIN	TRUE )
  set( QT_USE_QTXML		TRUE )
  set( QT_USE_QTOPENGL	TRUE )
  FIND_PACKAGE(Qt4)
  if(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
  else(QT4_FOUND)
    message(FATAL_ERROR "Qt4 not found. Please set QTDIR.")
  endif(QT4_FOUND)

  # temporary (very bad) hack for Qt4-Qt3D test (only for Qt-4.8.1-Qt3D-1.0-x86 under windows)...
  if (WITH_QT3D)
    find_library(QT3D_LIBRARIES
		NAMES Qt3D
		PATHS
		/usr/lib
		/usr/local/lib
		/sw/lib
		/opt/local/lib
		${QTDIR}/lib
		ENV LD_LIBRARY_PATH
		ENV LIBRARY_PATH
		PATH_SUFFIXES Qt3D
		)

    if(QT3D_LIBRARIES)
      MESSAGE(STATUS "Qt3D (for Qt4) libraries found: ${QT3D_LIBRARIES}")
    else(QT3D_LIBRARIES)
      MESSAGE(FATAL_ERROR "Qt3D (for Qt4) libraries not found.")
    endif(QT3D_LIBRARIES)
  endif(WITH_QT3D)
endif(WITH_QT5)
message("Qt version used : ${QTDIR}")

# Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

#if (APPLE) # for OpenGL and maybe other things
#  INCLUDE_DIRECTORIES(/usr/X11/include)
#endif (APPLE)

# Find ASSIMP
find_package(ASSIMP)
if(ASSIMP_FOUND)
  include_directories(${ASSIMP_INCLUDE_PATH})
else(ASSIMP_FOUND)
  message(FATAL_ERROR "ASSIMP not found. Please set ASSIMP_ROOT_DIR.")
endif(ASSIMP_FOUND)

# Find OpenSceneGraph
find_package(OpenSceneGraph)
if(OPENSCENEGRAPH_FOUND)
  include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
else(OPENSCENEGRAPH_FOUND)
  message(FATAL_ERROR "OpenSceneGraph not found. Please set OSG_DIR.")
endif(OPENSCENEGRAPH_FOUND)

# Find osg
find_package(osg)
if(OSG_FOUND)
  include_directories(${OSG_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osg not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgViewer
find_package(osgViewer)
if(OSGVIEWER_FOUND)
  include_directories(${OSGVIEWER_INCLUDE_DIR})
else(OSGVIEWER_FOUND)
  message(FATAL_ERROR "osgViewer not found. Please set OSG_DIR.")
endif(OSGVIEWER_FOUND)

# Find osgUtil
find_package(osgUtil)
if(OSGUTIL_FOUND)
  include_directories(${OSGUTIL_INCLUDE_DIR})
else(OSGUTIL_FOUND)
  message(FATAL_ERROR "osgUtil not found. Please set OSG_DIR.")
endif(OSGUTIL_FOUND)

# Find osgText
find_package(osgText)
if(OSGTEXT_FOUND)
  include_directories(${OSGTEXT_INCLUDE_DIR})
else(OSGTEXT_FOUND)
  message(FATAL_ERROR "osgText not found. Please set OSG_DIR.")
endif(OSGTEXT_FOUND)

# Find osgGA
find_package(osgGA)
if(OSGGA_FOUND)
  include_directories(${OSGGA_INCLUDE_DIR})
else(OSGGA_FOUND)
  message(FATAL_ERROR "osgGA not found. Please set OSG_DIR.")
endif(OSGGA_FOUND)

# Find osgDB
find_package(osgDB)
if(OSGDB_FOUND)
  include_directories(${OSGDB_INCLUDE_DIR})
else(OSGDB_FOUND)
  message(FATAL_ERROR "osgDB not found. Please set OSG_DIR.")
endif(OSGDB_FOUND)

# Find osgFX
find_package(osgFX)
if(OSGFX_FOUND)
  include_directories(${OSGFX_INCLUDE_DIR})
else(OSGFX_FOUND)
  message(FATAL_ERROR "osgFX not found. Please set OSG_DIR.")
endif(OSGFX_FOUND)

# Find osgShadow
find_package(osgShadow)
if(OSGSHADOW_FOUND)
  include_directories(${OSGSHADOW_INCLUDE_DIR})
else(OSGSHADOW_FOUND)
  message(FATAL_ERROR "osgShadow not found. Please set OSG_DIR.")
endif(OSGSHADOW_FOUND)

# Find osgWidget
find_package(osgWidget)
if(OSGWIDGET_FOUND)
  include_directories(${OSGWIDGET_INCLUDE_DIR})
else(OSGWIDGET_FOUND)
  message(FATAL_ERROR "osgWidget not found. Please set OSG_DIR.")
endif(OSGWIDGET_FOUND)

# Find osgQt (only for Linux and Mac OS X, not used with the "WIN32 x64 VCity binary kit" under Windows)
if (WIN32)
  SET( OSGQT_INCLUDE_DIR "" )
else(WIN32)
  find_package(osgQt)
  if(OSGQT_FOUND)
    include_directories(${OSGQT_INCLUDE_DIR})
  else(OSGQT_FOUND)
    message(FATAL_ERROR "osgQt not found. Please set OSG_DIR.")
  endif(OSGQT_FOUND)
endif(WIN32)

# Find OpenThreads
find_package(OpenThreads)
if(OPENTHREADS_FOUND)
  include_directories(${OPENTHREADS_INCLUDE_DIR})
else(OPENTHREADS_FOUND)
  message(FATAL_ERROR "OpenThreads not found. Please set OPENTHREADS_DIR.")
endif(OPENTHREADS_FOUND)

# Find GDAL
find_package(GDAL)
if(GDAL_FOUND)
  include_directories(${GDAL_INCLUDE_DIR})
else(GDAL_FOUND)
  message(FATAL_ERROR "GDAL not found. Please set GDAL_DIR or GDAL_ROOT.")
endif(GDAL_FOUND)

# Find LibXml2
find_package(LibXml2)
if(LIBXML2_FOUND)
  include_directories(${LIBXML2_INCLUDE_DIR})
  ADD_DEFINITIONS("-DUSE_LIBXML2")
else(LIBXML2_FOUND)
  message(FATAL_ERROR "LibXml2 not found.")
endif(LIBXML2_FOUND)

# Find Proj
#find_package(Proj)
#if(PROJ_FOUND)
#  include_directories(${PROJ_INCLUDE_DIR})
#else(PROJ_FOUND)
#  message(FATAL_ERROR "Proj not found.")
#endif(PROJ_FOUND)

# Find GEOS
#find_package(GEOS)
#if(GEOS_FOUND)
#  include_directories(${GEOS_INCLUDE_DIR})
#else(GEOS_FOUND)
#  message(FATAL_ERROR "GEOS not found.")
#endif(GEOS_FOUND)

# Find LASLIB
find_package(LASLIB)
if(LASLIB_FOUND)
  include_directories(${LASLIB_INCLUDE_DIR})
else(LASLIB_FOUND)
  message(FATAL_ERROR "LASLIB not found.")
endif(LASLIB_FOUND)

file(
  GLOB_RECURSE
  VCITY_GUI_Qt_SRC
  src/gui/moc/*.hpp
)
file(
  GLOB_RECURSE
  VCITY_GUI_Qt_UI
  ui/*.ui
)
SET( VCITY_GUI_Qt_RES
  resources/vcity.qrc
)

if (WITH_QT5)
  QT5_WRAP_UI(VCITY_GUI_UI_CPP ${VCITY_GUI_Qt_UI})
  QT5_ADD_RESOURCES(VCITY_GUI_RES_H ${VCITY_GUI_Qt_RES})
else(WITH_QT5)
  QT4_WRAP_CPP(VCITY_GUI_MOC_CPP ${VCITY_GUI_Qt_SRC})
  QT4_WRAP_UI(VCITY_GUI_UI_CPP ${VCITY_GUI_Qt_UI})
  QT4_ADD_RESOURCES(VCITY_GUI_RES_H ${VCITY_GUI_Qt_RES})
endif(WITH_QT5)

INCLUDE_DIRECTORIES( src src/libcitygml )

# General
file(
  GLOB_RECURSE
  VCITY_GUI_SRC
  src/*.cpp
)
SET( VCITY_GUI_SRC
  ${VCITY_GUI_SRC}
  ${VCITY_GUI_Qt_SRC}
  ${VCITY_GUI_UI_CPP}
  ${VCITY_GUI_RES_H}
)
if (WIN32)
  SET( VCITY_GUI_SRC_WIN src/gui/osgQt/GraphicsWindowQt.cxx )
  SET( VCITY_GUI_SRC ${VCITY_GUI_SRC} ${VCITY_GUI_SRC_WIN} )
endif(WIN32)
if (WITH_QT5)
else(WITH_QT5)
  SET( VCITY_GUI_SRC ${VCITY_GUI_SRC} ${VCITY_GUI_MOC_CPP} )
endif(WITH_QT5)

if (APPLE)
  set_source_files_properties(src/libcitygml/tesselator.cpp PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations") # for warnings: 'glu*' is deprecated...
endif (APPLE)

SET( VCITY_GUI_LIB
  ${ASSIMP_LIBRARY}
  ${OSG_LIBRARIES}
  ${OSGVIEWER_LIBRARIES}
  ${OSGUTIL_LIBRARIES}
  ${OSGTEXT_LIBRARIES}
  ${OSGGA_LIBRARIES}
  ${OSGDB_LIBRARIES}
  ${OSGFX_LIBRARIES}
  ${OSGSHADOW_LIBRARIES}
  ${OSGWIDGET_LIBRARIES}
  ${OSGQT_LIBRARIES}
  ${OPENTHREADS_LIBRARY}
  ${GDAL_LIBRARY}
  ${LIBXML2_LIBRARIES}
  #${PROJ_LIBRARY}
  #${GEOS_LIBRARY}
  ${LASLIB_LIBRARY}
  ${OPENGL_LIBRARIES}
)
if (WITH_QT5)
else(WITH_QT5)
  SET( VCITY_GUI_LIB ${VCITY_GUI_LIB} ${QT_LIBRARIES} )
  if (WITH_QT3D)
    SET( VCITY_GUI_LIB ${VCITY_GUI_LIB} ${QT3D_LIBRARIES} )
  endif(WITH_QT3D)
endif(WITH_QT5)

set(my_COMPILE_DEFINITIONS "")
if (WITH_QT5)
  set(my_COMPILE_DEFINITIONS ${my_COMPILE_DEFINITIONS} ";WITH_QT5")
endif(WITH_QT5)
if (WITH_QT3D)
  set(my_COMPILE_DEFINITIONS ${my_COMPILE_DEFINITIONS} ";WITH_QT3D")
endif(WITH_QT3D)

# --- EXECUTABLE ---

if (WITH_QT5)
  if( NOT DEFINED QT_PLUGINS_DIR )
    set( QT_PLUGINS_DIR "${QT5_DIR}/plugins" )
  endif( NOT DEFINED QT_PLUGINS_DIR )
endif(WITH_QT5)

IF(WIN32)
 #--------------------------------------------------------------------------------
 # Copy Qt plugins to 'Debug and Release' directories and configure qt.conf file
 #--------------------------------------------------------------------------------
 file(GLOB qtplugin_dirs RELATIVE  "${QT_PLUGINS_DIR}"  "${QT_PLUGINS_DIR}/imageformats*"  "${QT_PLUGINS_DIR}/platforms*")
 file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/Debug/QtPlugins")
 file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/Release/QtPlugins")
 foreach (qtplugin ${qtplugin_dirs})
    file(COPY "${QT_PLUGINS_DIR}/${qtplugin}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/Debug/QtPlugins")
    file(COPY "${QT_PLUGINS_DIR}/${qtplugin}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/Release/QtPlugins")
 endforeach ()
 file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/Debug/qt.conf" "[Paths]\nPlugins = QtPlugins")
 file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/Release/qt.conf" "[Paths]\nPlugins = QtPlugins")
ENDIF(WIN32)

# For Windows set the ico file with the rc file
IF(WIN32)
  SET(VCITY_GUI_ICON resources/vcity.rc)
ENDIF(WIN32)

# For Apple set the icns file containing icons
IF(APPLE)
  # set how it shows up in the Info.plist file
  SET(MACOSX_BUNDLE_ICON_FILE vcity.icns)
  # set where in the bundle to put the icns file
  SET_SOURCE_FILES_PROPERTIES(resources/vcity.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
  # include the icns file in the target
  SET(VCITY_GUI_ICON resources/vcity.icns)
ENDIF(APPLE)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    # Linux specific code
    SET(LINUX_FLAGS -lpthread)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

add_executable(${PRJ_NAME} MACOSX_BUNDLE ${VCITY_GUI_SRC} ${VCITY_GUI_ICON})
target_link_libraries (${PRJ_NAME} ${LINUX_FLAGS} ${VCITY_GUI_LIB} raytracing)
if (WITH_QT5)
  ADD_DEFINITIONS("-DWITH_QT5")
  QT5_USE_MODULES( ${PRJ_NAME} Widgets Xml OpenGL )
  if (WITH_QT3D)
    QT5_USE_MODULES( ${PRJ_NAME} 3D )
  endif(WITH_QT3D)
endif(WITH_QT5)
SET_TARGET_PROPERTIES(${PRJ_NAME} PROPERTIES COMPILE_DEFINITIONS "${my_COMPILE_DEFINITIONS}")

add_executable(CityGMLCut src/utils/cmdline/cityGMLCut.cxx src/utils/cmdline/triangulate.cxx)
target_link_libraries (CityGMLCut ${VCITY_GUI_LIB})
if (WITH_QT5)
  ADD_DEFINITIONS("-DWITH_QT5")
  QT5_USE_MODULES( CityGMLCut Widgets Xml OpenGL )
  if (WITH_QT3D)
    QT5_USE_MODULES( CityGMLCut 3D )
  endif(WITH_QT3D)
endif(WITH_QT5)
SET_TARGET_PROPERTIES(CityGMLCut PROPERTIES COMPILE_DEFINITIONS "${my_COMPILE_DEFINITIONS}")

################
# 3DUSE plugins
################

if (WITH_QT5) # For the plugins, instruct CMake to not run moc automatically when needed
 set(CMAKE_AUTOMOC OFF)
 QT5_WRAP_CPP(VCITY_GUI_MOC_CPP ${VCITY_GUI_Qt_SRC})
 SET( VCITY_GUI_SRC ${VCITY_GUI_SRC} ${VCITY_GUI_MOC_CPP} )
endif(WITH_QT5)

# --> CityGMLCutQtPlugin : QtPlugin [BEGIN]
OPTION(BUILD_CityGMLCutQtPlugin "BUILD CityGMLCutQtPlugin " ON)
if (BUILD_CityGMLCutQtPlugin)
	set(CityGMLCut_Qt_Plugin_HEADER src/plugins/CityGMLCutQtPlugin/CityGMLCutQtPlugin.h src/plugins/CityGMLCutQtPlugin/Dialogs/DialogCityGMLCut1.h)
	set(CityGMLCut_Qt_Plugin_UI src/plugins/CityGMLCutQtPlugin/Dialogs/DialogCityGMLCut1.ui)
	if (WITH_QT5)
	  ADD_DEFINITIONS("-DWITH_QT5")
	  QT5_WRAP_CPP(CityGMLCut_Qt_Plugin_MOC_CPP ${CityGMLCut_Qt_Plugin_HEADER})
	  QT5_WRAP_UI(CityGMLCut_Qt_Plugin_UI_CPP ${CityGMLCut_Qt_Plugin_UI})
	  set(CityGMLCut_Qt_Plugin_SRC ${CityGMLCut_Qt_Plugin_SRC} ${CityGMLCut_Qt_Plugin_MOC_CPP} ${CityGMLCut_Qt_Plugin_UI_CPP})
	else(WITH_QT5) 
	  QT4_WRAP_CPP(CityGMLCut_Qt_Plugin_MOC_CPP ${CityGMLCut_Qt_Plugin_HEADER})
	  QT4_WRAP_UI(CityGMLCut_Qt_Plugin_UI_CPP ${CityGMLCut_Qt_Plugin_UI})
	  set(CityGMLCut_Qt_Plugin_SRC ${CityGMLCut_Qt_Plugin_SRC} ${CityGMLCut_Qt_Plugin_MOC_CPP} ${CityGMLCut_Qt_Plugin_UI_CPP})
	  set(CityGMLCut_Qt_Plugin_LIB ${CityGMLCut_Qt_Plugin_LIB} ${QT_LIBRARIES})
	endif(WITH_QT5)

	add_library(CityGMLCutQtPlugin SHARED src/plugins/CityGMLCutQtPlugin/CityGMLCutQtPlugin.cxx src/plugins/CityGMLCutQtPlugin/Dialogs/DialogCityGMLCut1.cxx
				${CityGMLCut_Qt_Plugin_HEADER} ${CityGMLCut_Qt_Plugin_SRC}
				${VCITY_GUI_SRC_WIN} src/utils/cmdline/cityGMLCut.cxx src/utils/cmdline/triangulate.cxx) # if need, add cxx files here before the ')'
	target_link_libraries (CityGMLCutQtPlugin ${CityGMLCut_Qt_Plugin_LIB} ${VCITY_GUI_LIB})
	if (WITH_QT5)
	  QT5_USE_MODULES(CityGMLCutQtPlugin Core Widgets OpenGL)
	endif(WITH_QT5)
endif (BUILD_CityGMLCutQtPlugin)
# --> CityGMLCutQtPlugin : QtPlugin [END]

# --> CityGMLEmptyQtPlugin : QtPlugin [BEGIN]
OPTION(BUILD_CityGMLEmptyQtPlugin "BUILD CityGMLEmptyQtPlugin " ON)
if (BUILD_CityGMLEmptyQtPlugin)
	set(CityGMLEmpty_Qt_Plugin_HEADER src/plugins/CityGMLEmptyQtPlugin/CityGMLEmptyQtPlugin.h src/plugins/CityGMLEmptyQtPlugin/Dialogs/DialogCityGMLEmpty1.h)
	set(CityGMLEmpty_Qt_Plugin_UI src/plugins/CityGMLEmptyQtPlugin/Dialogs/DialogCityGMLEmpty1.ui)
	if (WITH_QT5)
	  ADD_DEFINITIONS("-DWITH_QT5")
	  QT5_WRAP_CPP(CityGMLEmpty_Qt_Plugin_MOC_CPP ${CityGMLEmpty_Qt_Plugin_HEADER})
	  QT5_WRAP_UI(CityGMLEmpty_Qt_Plugin_UI_CPP ${CityGMLEmpty_Qt_Plugin_UI})
	  set(CityGMLEmpty_Qt_Plugin_SRC ${CityGMLEmpty_Qt_Plugin_SRC} ${CityGMLEmpty_Qt_Plugin_MOC_CPP} ${CityGMLEmpty_Qt_Plugin_UI_CPP})
	else(WITH_QT5) 
	  QT4_WRAP_CPP(CityGMLEmpty_Qt_Plugin_MOC_CPP ${CityGMLEmpty_Qt_Plugin_HEADER})
	  QT4_WRAP_UI(CityGMLEmpty_Qt_Plugin_UI_CPP ${CityGMLEmpty_Qt_Plugin_UI})
	  set(CityGMLEmpty_Qt_Plugin_SRC ${CityGMLEmpty_Qt_Plugin_SRC} ${CityGMLEmpty_Qt_Plugin_MOC_CPP} ${CityGMLEmpty_Qt_Plugin_UI_CPP})
	  set(CityGMLEmpty_Qt_Plugin_LIB ${CityGMLEmpty_Qt_Plugin_LIB} ${QT_LIBRARIES})
	endif(WITH_QT5)

	add_library(CityGMLEmptyQtPlugin SHARED src/plugins/CityGMLEmptyQtPlugin/CityGMLEmptyQtPlugin.cxx src/plugins/CityGMLEmptyQtPlugin/Dialogs/DialogCityGMLEmpty1.cxx
				${CityGMLEmpty_Qt_Plugin_HEADER} ${CityGMLEmpty_Qt_Plugin_SRC}
				${VCITY_GUI_SRC_WIN}) # if need, add cxx files here before the ')'
	target_link_libraries (CityGMLEmptyQtPlugin ${CityGMLEmpty_Qt_Plugin_LIB} ${VCITY_GUI_LIB})
	if (WITH_QT5)
	  QT5_USE_MODULES(CityGMLEmptyQtPlugin Core Widgets OpenGL)
	endif(WITH_QT5)
endif (BUILD_CityGMLEmptyQtPlugin)
# --> CityGMLEmptyQtPlugin : QtPlugin [END]

# --> CityGMLVisibiliteQtPlugin : QtPlugin [BEGIN]
OPTION(BUILD_CityGMLVisibiliteQtPlugin "BUILD CityGMLVisibiliteQtPlugin " OFF)
if (BUILD_CityGMLVisibiliteQtPlugin)
  file(
    GLOB_RECURSE
    CityGMLVisibilite_Qt_Plugin_SRC
    src/plugins/CityGMLVisibiliteQtPlugin/*.cxx
  )
	set(CityGMLVisibilite_Qt_Plugin_HEADER src/plugins/CityGMLVisibiliteQtPlugin/CityGMLVisibiliteQtPlugin.h src/plugins/CityGMLVisibiliteQtPlugin/Dialogs/dialogVisibilite.hpp)
	set(CityGMLVisibilite_Qt_Plugin_UI src/plugins/CityGMLVisibiliteQtPlugin/Dialogs/dialogVisibilite.ui)
	if (WITH_QT5)
	  ADD_DEFINITIONS("-DWITH_QT5")
	  QT5_WRAP_CPP(CityGMLVisibilite_Qt_Plugin_MOC_CPP ${CityGMLVisibilite_Qt_Plugin_HEADER})
	  QT5_WRAP_UI(CityGMLVisibilite_Qt_Plugin_UI_CPP ${CityGMLVisibilite_Qt_Plugin_UI})
	  set(CityGMLVisibilite_Qt_Plugin_SRC ${CityGMLVisibilite_Qt_Plugin_SRC} ${CityGMLVisibilite_Qt_Plugin_MOC_CPP} ${CityGMLVisibilite_Qt_Plugin_UI_CPP})
	else(WITH_QT5) 
	  QT4_WRAP_CPP(CityGMLVisibilite_Qt_Plugin_MOC_CPP ${CityGMLVisibilite_Qt_Plugin_HEADER})
	  QT4_WRAP_UI(CityGMLVisibilite_Qt_Plugin_UI_CPP ${CityGMLVisibilite_Qt_Plugin_UI})
	  set(CityGMLVisibilite_Qt_Plugin_SRC ${CityGMLVisibilite_Qt_Plugin_SRC} ${CityGMLVisibilite_Qt_Plugin_MOC_CPP} ${CityGMLVisibilite_Qt_Plugin_UI_CPP})
	  set(CityGMLVisibilite_Qt_Plugin_LIB ${CityGMLVisibilite_Qt_Plugin_LIB} ${QT_LIBRARIES})
	endif(WITH_QT5)

	add_library(CityGMLVisibiliteQtPlugin SHARED src/plugins/CityGMLVisibiliteQtPlugin/CityGMLVisibiliteQtPlugin.cxx src/plugins/CityGMLVisibiliteQtPlugin/Dialogs/dialogVisibilite.cxx
				${CityGMLVisibilite_Qt_Plugin_HEADER} ${CityGMLVisibilite_Qt_Plugin_SRC}
				${VCITY_GUI_SRC}) # if need, add cxx files here before the ')'
	target_link_libraries (CityGMLVisibiliteQtPlugin ${LINUX_FLAGS} ${CityGMLVisibilite_Qt_Plugin_LIB} ${VCITY_GUI_LIB})
	if (WITH_QT5)
	  QT5_USE_MODULES(CityGMLVisibiliteQtPlugin Core Widgets OpenGL)
	endif(WITH_QT5)
endif (BUILD_CityGMLVisibiliteQtPlugin)
# --> CityGMLVisibiliteQtPlugin : QtPlugin [END]

################
# 3DUSE plugins
################

	#--------------------------------------------------------------------------------
	# Now the installation stuff : http://www.cmake.org/Wiki/BundleUtilitiesExample
	#--------------------------------------------------------------------------------
	SET(plugin_dest_dir .)
	SET(qtconf_dest_dir .)
	SET(APPS "\${CMAKE_INSTALL_PREFIX}/${PRJ_NAME}")
	IF(APPLE)
		SET(plugin_dest_dir ${PRJ_NAME}.app/Contents/MacOS)
		SET(qtconf_dest_dir ${PRJ_NAME}.app/Contents/Resources)
		SET(APPS "\${CMAKE_INSTALL_PREFIX}/${PRJ_NAME}.app")
	ENDIF(APPLE)
	IF(WIN32)
		SET(APPS "\${CMAKE_INSTALL_PREFIX}/${PRJ_NAME}.exe")
	ENDIF(WIN32)

	#--------------------------------------------------------------------------------
	# Install the application, on Apple, the bundle is at the root of the
	# install tree, and on other platforms it'll go to the same directory.
	################
	# 3DUSE plugins
	################
	if (BUILD_CityGMLCutQtPlugin)
		set(CityGMLCutQtPlugin "CityGMLCutQtPlugin")
	endif (BUILD_CityGMLCutQtPlugin)
	if (BUILD_CityGMLEmptyQtPlugin)
		set(CityGMLEmptyQtPlugin "CityGMLEmptyQtPlugin")
	endif (BUILD_CityGMLEmptyQtPlugin)
	if (BUILD_CityGMLVisibiliteQtPlugin)
		set(CityGMLVisibiliteQtPlugin "CityGMLVisibiliteQtPlugin")
	endif (BUILD_CityGMLVisibiliteQtPlugin)
	################
	# 3DUSE plugins
	################
	INSTALL(TARGETS ${PRJ_NAME} CityGMLCut ${CityGMLCutQtPlugin} ${CityGMLEmptyQtPlugin} ${CityGMLVisibiliteQtPlugin} # add your plugin installation here
		BUNDLE DESTINATION . COMPONENT ${PRJ_NAME}
		RUNTIME DESTINATION . COMPONENT ${PRJ_NAME}
		LIBRARY DESTINATION . COMPONENT ${PRJ_NAME}
	)

	#--------------------------------------------------------------------------------
	# Install needed Qt plugins by copying directories from the qt installation
	# One can cull what gets copied by using 'REGEX "..." EXCLUDE'
	INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats" DESTINATION ${plugin_dest_dir}/plugins COMPONENT ${PRJ_NAME})
	if (WITH_QT5)
		INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/platforms" DESTINATION ${plugin_dest_dir}/plugins COMPONENT ${PRJ_NAME})
	endif(WITH_QT5)

	# OSG plugins
	if(OPENSCENEGRAPH_FOUND)
		INCLUDE(BundleOSGPlugins)
		install_osg_plugins( osg_plugins )
		MESSAGE( STATUS "OSG plugins: " ${osg_plugins} )
	endif(OPENSCENEGRAPH_FOUND)

	#--------------------------------------------------------------------------------
	# install a qt.conf file
	# this inserts some cmake code into the install script to write the file
	INSTALL(CODE "
		file(WRITE \"\${CMAKE_INSTALL_PREFIX}/${qtconf_dest_dir}/qt.conf\" \"[Paths]\nPlugins = plugins\")
		" COMPONENT ${PRJ_NAME})
	IF(APPLE)
		INSTALL(CODE "
			file(WRITE \"\${CMAKE_INSTALL_PREFIX}/${qtconf_dest_dir}/qt.conf\" \"[Paths]\nPlugins = MacOS/plugins\")
			" COMPONENT ${PRJ_NAME})
	ENDIF(APPLE)

	# install qt_menu.nib (APPLE)
	IF(APPLE)
		IF(EXISTS ${QT_LIBRARY_DIR}/Resources/qt_menu.nib)
			INSTALL(CODE "execute_process(COMMAND cp -R \"${QT_LIBRARY_DIR}/Resources/qt_menu.nib\" \"${qtconf_dest_dir}\"
				 WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX})")
		ELSE(EXISTS ${QT_LIBRARY_DIR}/Resources/qt_menu.nib)
			IF(EXISTS ${QT_LIBRARY_DIR}/QtGui.framework/Resources/qt_menu.nib)
				INSTALL(CODE "execute_process(COMMAND cp -R \"${QT_LIBRARY_DIR}/QtGui.framework/Resources/qt_menu.nib\" \"${qtconf_dest_dir}\"
					 WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX})")
			ENDIF(EXISTS ${QT_LIBRARY_DIR}/QtGui.framework/Resources/qt_menu.nib)
		ENDIF(EXISTS ${QT_LIBRARY_DIR}/Resources/qt_menu.nib)
	ENDIF(APPLE)

	#--------------------------------------------------------------------------------
	# Use BundleUtilities to get all other dependencies for the application to work.
	# It takes a bundle or executable along with possible plugins and inspects it
	# for dependencies.  If they are not system dependencies, they are copied.

	# directories to look for dependencies
	SET(DIRS ${QT_LIBRARY_DIRS})

	# Now the work of copying dependencies into the bundle/package
	# The quotes are escaped and variables to use at install time have their $ escaped
	# An alternative is the do a configure_file() on a script and use install(SCRIPT  ...).
	# Note that the image plugins depend on QtSvg and QtXml, and it got those copied
	# over.
	INSTALL(CODE "
		file(GLOB_RECURSE PLUGINS
		\"\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\" \"\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/osgPlugins-${OPENSCENEGRAPH_VERSION}/osgdb_*${CMAKE_SHARED_LIBRARY_SUFFIX}\" \"\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/*component_*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
		include(BundleUtilities)
		fixup_bundle(\"${APPS}\" \"\${PLUGINS}\" \"${DIRS}\")
		" COMPONENT ${PRJ_NAME})

	# CPack Configuration
	set(CPACK_PACKAGE_NAME ${PRJ_NAME})
	set(CPACK_PACKAGE_VERSION ${PRJ_VERSION})
	set(CPACK_PACKAGE_VENDOR "Liris / CNRS")
	#set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README)
	#set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/licence_gpl_v3.txt)

	# To Create a package, one can run "cpack -G DragNDrop CPackConfig.cmake" on Mac OS X
	# where CPackConfig.cmake is created by including CPack
	# And then there's ways to customize this as well
	IF(APPLE)
		set(CPACK_BINARY_DRAGNDROP ON)
		set(CPACK_BINARY_TGZ "OFF")
		set(CPACK_BINARY_STGZ "OFF")
		set(CPACK_BINARY_PACKAGEMAKER "OFF")
		include(CPack)
	ENDIF(APPLE)

	IF(WIN32)
		set(CPACK_GENERATOR "NSIS64")
		set(CPACK_NSIS_MENU_LINKS "${PRJ_NAME}" "${PRJ_NAME}")
		#MESSAGE( STATUS "PRJ_NAME: " ${PRJ_NAME} )
		include(CPack)
	ENDIF(WIN32)
	#--------------------------------------------------------------------------------
	# Now the installation stuff : end
	#--------------------------------------------------------------------------------

add_subdirectory(src)
