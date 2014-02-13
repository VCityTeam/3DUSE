if (WITH_QT5)
  cmake_minimum_required(VERSION 2.8.9) # because 2.8.8 has a bug with QT5_USE_MODULES 
endif(WITH_QT5)

if (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  IF(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)
  ENDIF()
endif (COMMAND cmake_policy)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

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
# If you are compiling on Windows, the preprocessor symbols 'min' or 'max' are defined. Do #define NOMINMAX to prevent windows.h from defining these symbols.
if(MSVC)
	SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D \"NOMINMAX\"")
endif()
#------------------------------------------------------------------------------

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# WIN32 x64 VCity binary kit
IF(WIN32)
	if( NOT DEFINED VCITY_KIT_ROOT )
		message(FATAL_ERROR "VCITY_KIT_ROOT not set.  Please set VCITY_KIT_ROOT.")
	endif( NOT DEFINED VCITY_KIT_ROOT )

	if(WITH_QT4)
		# with qt4
		set(WITH_QT5			"OFF")
		set(QTDIR				${VCITY_KIT_ROOT}/Qt/qt-4.8.5-x64-msvc2012)
	else(WITH_QT4)
		# with qt5
		set(WITH_QT5			"ON")
		set(QT5_DIR				${VCITY_KIT_ROOT}/Qt/Qt5.1.0/5.1.0/msvc2012_64_opengl)
	endif(WITH_QT4)

	set(OSG_DIR					${VCITY_KIT_ROOT}/osg/osg-3.2.0-bin-without-qt)

	set(GDAL_INCLUDE_DIR		${OSG_DIR}/../3rdParty/x64/include)
	set(GDAL_LIBRARY			${OSG_DIR}/../3rdParty/x64/lib/gdal_i.lib)

	set(LIBXML2_INCLUDE_DIR		${OSG_DIR}/../3rdParty/x64/include/libxml2)
	set(LIBXML2_LIBRARIES		${OSG_DIR}/../3rdParty/x64/lib/libxml2.lib) #libxml2D.lib

	set(PROJ_INCLUDE_DIR		${OSG_DIR}/../3rdParty/x64/include)
	set(PROJ_LIBRARY			${OSG_DIR}/../3rdParty/x64/lib/proj.lib)
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

# Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

#if (APPLE) # for OpenGL and maybe other things
#  INCLUDE_DIRECTORIES(/usr/X11/include)
#endif (APPLE)

# Find osg
find_package(osg)
if(OSG_FOUND)
  include_directories(${OSG_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osg not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgViewer
find_package(osgViewer)
if(OSG_FOUND)
  include_directories(${OSGVIEWER_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osgViewer not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgUtil
find_package(osgUtil)
if(OSG_FOUND)
  include_directories(${OSGUTIL_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osgUtil not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgText
find_package(osgText)
if(OSG_FOUND)
  include_directories(${OSGTEXT_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osgText not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgGA
find_package(osgGA)
if(OSG_FOUND)
  include_directories(${OSGGA_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osgGA not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgDB
find_package(osgDB)
if(OSG_FOUND)
  include_directories(${OSGDB_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osgDB not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgFX
find_package(osgFX)
if(OSG_FOUND)
  include_directories(${OSGFX_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osgFX not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgShadow
find_package(osgShadow)
if(OSG_FOUND)
  include_directories(${OSGSHADOW_INCLUDE_DIR})
else(OSG_FOUND)
  message(FATAL_ERROR "osgShadow not found. Please set OSG_DIR.")
endif(OSG_FOUND)

# Find osgQt (only for Linux and Mac OS X, not used with the "WIN32 x64 VCity binary kit" under Windows)
if (WIN32)
  SET( OSGQT_INCLUDE_DIR "" )
else(WIN32)
  find_package(osgQt)
  if(OSG_FOUND)
    include_directories(${OSGQT_INCLUDE_DIR})
  else(OSG_FOUND)
    message(FATAL_ERROR "osgQt not found. Please set OSG_DIR.")
  endif(OSG_FOUND)
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
find_package(Proj)
if(PROJ_FOUND)
  include_directories(${PROJ_INCLUDE_DIR})
else(PROJ_FOUND)
  message(FATAL_ERROR "Proj not found.")
endif(PROJ_FOUND)

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
  SET( VCITY_GUI_SRC ${VCITY_GUI_SRC} src/gui/osgQt/GraphicsWindowQt.cxx )
endif(WIN32)
if (WITH_QT5)
else(WITH_QT5)
  SET( VCITY_GUI_SRC ${VCITY_GUI_SRC} ${VCITY_GUI_MOC_CPP} )
endif(WITH_QT5)
 
SET( VCITY_GUI_LIB
  ${OSG_LIBRARIES}
  ${OSGVIEWER_LIBRARIES}
  ${OSGUTIL_LIBRARIES}
  ${OSGTEXT_LIBRARIES}
  ${OSGGA_LIBRARIES}
  ${OSGDB_LIBRARIES}
  ${OSGFX_LIBRARIES}
  ${OSGSHADOW_LIBRARIES}
  ${OSGQT_LIBRARIES}
  ${OPENTHREADS_LIBRARY}
  ${GDAL_LIBRARY}
  ${LIBXML2_LIBRARIES}
  ${PROJ_LIBRARY}
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
    set( QT_PLUGINS_DIR "${QT5_DIR}/plugins/" )
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
    file(COPY "${QT_PLUGINS_DIR}/${qtplugin}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/Debug/QtPlugins/")
    file(COPY "${QT_PLUGINS_DIR}/${qtplugin}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/Release/QtPlugins/")
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

ADD_EXECUTABLE(VCity MACOSX_BUNDLE ${VCITY_GUI_SRC} ${VCITY_GUI_ICON})
target_link_libraries (VCity ${VCITY_GUI_LIB})
if (WITH_QT5)
  QT5_USE_MODULES( VCity Widgets Xml OpenGL )
  if (WITH_QT3D)
    QT5_USE_MODULES( VCity 3D )
  endif(WITH_QT3D)
endif(WITH_QT5)
SET_TARGET_PROPERTIES(VCity PROPERTIES COMPILE_DEFINITIONS "${my_COMPILE_DEFINITIONS}")