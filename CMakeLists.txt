project( 3DUSE )

# FIXME Under construction
# option( BUILD_GUI_QT4 "Build the Graphical User Interface with QT4" OFF )
# option( BUILD_GUI_QT5 "Build the Graphical User Interface with QT5" ON )

# FIXME: a pre-requisite of discussion on cmake version (below) is to discuss
# the supported platforms and distros (which constrains the cmake available/
# possible versions. Availabilty:
#  *  cmake 2.8.9 is available for Ubuntu 14.04, debian 7, fedora 21,
#     OpenSUSE 13.1...
# FIXME: establish cmake minimum version with the following constraints:
#  * Ubuntu 14.04 (current Travis image) still uses 2.8.12 (cmake 3.0 is
#    thus too high)
#  * QT5_USE_MODULES has a bug for cmake version 2.8.8
# Proposed version: requiring anything in the range 2.8.9 to 2.8.12 would
# make the job. Go for the higher version within this range i.e. 2.8.12 ?
cmake_minimum_required(VERSION 2.8.12)

if ( WITH_QT5 )
  cmake_minimum_required(VERSION 2.8.9)
endif ()

include( cmake/3DusePolicies.cmake )

enable_testing()

# Project configuration
set(PRJ_NAME 3DUSE)

SET(PRJ_VERSION_MAJOR 0)
SET(PRJ_VERSION_MINOR 3)
SET(PRJ_VERSION_PATCH 2)
SET(PRJ_VERSION "${PRJ_VERSION_MAJOR}.${PRJ_VERSION_MINOR}.${PRJ_VERSION_PATCH}")
message(STATUS "${PRJ_NAME} ${PRJ_VERSION}")

# FIXME: this will override any user defined value of CMAKE_INSTALL_PREFIX
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/${PRJ_NAME}_deploy_${PRJ_VERSION}")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Set Release as default build target
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: Debug, Release."
      FORCE)
endif ()
message(STATUS "Build type : ${CMAKE_BUILD_TYPE}")

##################################################
# Platform/Compiler specific options

##### GNUCXX compiler specifics
if( CMAKE_COMPILER_IS_GNUCXX )
  set( ENABLE_CXX11 "-std=c++11" )

  execute_process( COMMAND "${CMAKE_CXX_COMPILER} -dumpversion"
                   OUTPUT_VARIABLE GCC_VERSION )
  if( GCC_VERSION LESS 4.7 )
    set( ENABLE_CXX11 "-std=c++0x" )
  endif()

  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ENABLE_CXX11}" )
  message(STATUS "GNUCXX compiler detected (with ${ENABLE_CXX11} activated)")
endif()

##### Clang compiler specifics (export CXX=clang++ CC=clang)
if( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
  set( ENABLE_CXX11 "-std=c++11" )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ENABLE_CXX11} -Qunused-arguments" )
  set( CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${ENABLE_CXX11}   -Qunused-arguments" )
  message( STATUS "Clang compiler detected (with ${ENABLE_CXX11} activated)" )
endif()

##### Micro$oft Visual C++ compiler specifics (aka VC++)
if(MSVC)
  # /MP sets the usage of multiple compilation units (cl.exe) ; without any
  #     value, as argument the /MP compile flag will be set automatically
  #     according to available number of main threads
  # NOMINMAX (preprocessing flag): by default VC++ preprocessor defines the 
  #     'min' and 'max' symbols. Setting the NOMINMAX preprocessing symbol
  #     inhibits this default behavior i.e. it will prevent windows.h from
  #     defining the 'min' and 'max' symbols (and avoid further collisions).
  # FIXME: discuss the following with MTO
  # /FORCE:MULTIPLE
  #      - has an effect on osgDB ifstream / ofstream conflict ?
  #      - /FORCE:MULTIPLE should be avoided (e.g. /INCREMENTAL is disabled
  #        because of it)
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /D \"NOMINMAX\"")

  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "19")
    set(KIT_VS2015 YES)
  endif()

  include( "${CMAKE_SOURCE_DIR}/cmake/msvc/kit.cmake" )
 
  # For each available configuration type (refer to CMAKE_CONFIGURATION_TYPES) 
  # gather all the binaries (executables, archives and dynamic libraries) within
  # the same (configuration dependent) directory.
  # The objective is to have the executables to sit side by side with the dlls
  # on which they depend on for the loader to find them at runtime.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG} )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG} )
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG} )
  endforeach()
endif()

##### OSX specifics (Apple)
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON) # see cmake POLICY CMP0042
endif(APPLE)

############################## Package (dependencies) detection
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
endif(WITH_QT5)
message(STATUS "Qt version used : ${QTDIR}")

# Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

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

# Find osg (which is a sub-module of OpenSceneGraph)
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

# Find osgQt
# FIXME: assert the following with MTO
# Not surprisingly OSG-QT depends both on OSG and QT. Depending on
# the platform/distribution the packagers choose some QT version
# (4 or 5) to in order to package OSG-QT. 3DUse offers support for
# both QT4 and QT5. This can produce collisions when e.g. 3DUSe is
# build against QT5 but the OSG-QT package is against QT4. Solving
# this is done by embedding the (various versions of) OSG-QT within
# 3DUse (refer to src/gui/osgQt directory). Then when required (this
# is the difficult part) either the native OSG-QT package is used or
# the embedded version is used.
# The trick is realised with the usage of the variables
# - OSGQT_INCLUDE_DIR (either set by find_package or manually)
# - VCITY_GUI_SRC_WIN that is either empty (for the native package) or points
#   to the ad-hoc version of the sources
# With QT4 the split goes:
#  - Embedded code used for: 
#     - MacOS AND TRAVIS     
#     - Win32
#  - Native package for:
#     - MacOS (not on Travis) 
#     - Linux
# With QT5 the split goes:
#  - Embedded code used for:  FIXME: ask MTO ?
#  - Native package for:      FIXME: ask MTO ?
#  
# FIXME: why consider Travis as being special case that needs to be
# flagged through an environement variable ? Shouldn't Travis be
# a platform like another one i.e. indirectly specified through its
# compiler and version of QT4 or QT5 ?

if (WIN32)
  SET( OSGQT_INCLUDE_DIR "" )
elseif(DEFINED ENV{TRAVIS})
  if (APPLE)
    set( OSGQT_INCLUDE_DIR "src/gui/osgQt/osg34/include" )
    include_directories(${OSGQT_INCLUDE_DIR})
  else()
    find_package(osgQt)
    if(OSGQT_FOUND)
      include_directories(${OSGQT_INCLUDE_DIR})
    else(OSGQT_FOUND)
      message(FATAL_ERROR "osgQt not found. Please set OSG_DIR.")
    endif(OSGQT_FOUND)
  endif()
else()
  find_package(osgQt)
  if(OSGQT_FOUND)
    include_directories(${OSGQT_INCLUDE_DIR})
  else(OSGQT_FOUND)
    message(FATAL_ERROR "osgQt not found. Please set OSG_DIR.")
  endif(OSGQT_FOUND)
endif()

# Find OpenThreads (which is also an OSG sub-module)
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
  QT5_WRAP_UI(       VCITY_GUI_UI_CPP ${VCITY_GUI_Qt_UI})
  QT5_ADD_RESOURCES( VCITY_GUI_RES_CPP ${VCITY_GUI_Qt_RES})
else(WITH_QT5)
  QT4_WRAP_CPP(      VCITY_GUI_MOC_CPP ${VCITY_GUI_Qt_SRC})
  QT4_WRAP_UI(       VCITY_GUI_UI_CPP  ${VCITY_GUI_Qt_UI})
  QT4_ADD_RESOURCES( VCITY_GUI_RES_CPP ${VCITY_GUI_Qt_RES})
endif(WITH_QT5)

##### Local libraries
# The following variable definitions add syntactic sugat in an attempt
# to unify notations between external libraries and internal (defined
# within this project) libraries. 
# FIXME: those definition should be deported to their respective
# sub-directories.

# CORE
set( VCITYCORE_INCLUDE_DIR
  ${CMAKE_SOURCE_DIR}/src/DataStructures/DEM
  ${CMAKE_BINARY_DIR}/src                   # for generated vcitycore_export.h
)
set( VCITYCORE_LIBRARY vcitycore )

# CITYGML--> CORE
set( CITYGML_INCLUDE_DIR
  ${CMAKE_SOURCE_DIR}/src/libcitygml
  ${CMAKE_SOURCE_DIR}/src/libcitygml/export # FIXME: remove once flatten out 
  ${CMAKE_SOURCE_DIR}/src/libcitygml/import # FIXME: remove once flatten out
  ${CMAKE_SOURCE_DIR}/src/libcitygml/utils  # FIXME: will become a library
  ${CMAKE_BINARY_DIR}/src/libcitygml        # for generated citygml_export.h
)
set( CITYGML_LIBRARIES citygml )

# CITYGML UTILS--> CITYGML
set( CITYGMLUTILS_INCLUDE_DIR
  ${CMAKE_SOURCE_DIR}/src/libcitygml/utils
  ${CMAKE_BINARY_DIR}/src/libcitygml/utils  # generated citygmlutils_export.h
)
set( CITYGMLUTILS_LIBRARIES citygmlutils )

# FILTERS--> CORE, CITYGML, CITYGMLUTILS
set( FILTERS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/libfilters )
set( FILTERS_LIBRARIES   filters )


# FIXME: inquire on the necessity of the following line
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)


################### General
set( VCITY_GUI_SRC
### The core
  src/core/RayBox.cpp
  src/core/abstractlayer.cpp
  src/core/algo.cpp
  src/core/algo2.cpp
  src/core/application.cpp
  src/core/cell.cpp
  src/core/controller.cpp
  src/core/dataprofile.cpp
  src/core/layerAssimp.cpp
  src/core/layerCityGML.cpp
  src/core/layerLas.cpp
  src/core/layerMnt.cpp
  src/core/layerShp.cpp
  src/core/layerTiledCityGML.cpp
  src/core/scene.cpp
  src/core/settings.cpp
  src/core/tools/log.cpp
### The gui  
  src/gui/applicationGui.cpp
  src/gui/controllerGui.cpp
  src/gui/dialogAbout.cpp
  src/gui/dialogAddBuilding.cpp
  src/gui/dialogAddLayer.cpp
  src/gui/dialogDoc.cpp
  src/gui/dialogDynState.cpp
  src/gui/dialogEditAssimpNode.cpp
  src/gui/dialogEditBldg.cpp
  src/gui/dialogEditLayer.cpp
  src/gui/dialogEditTile.cpp
  src/gui/dialogLink.cpp
  src/gui/dialogLoadBBox.cpp
  src/gui/dialogSettings.cpp
  src/gui/dialogShpTool.cpp
  src/gui/dialogState.cpp
  src/gui/dialogTag.cpp
  src/gui/dialogTilingCityGML.cpp
  src/gui/dialogYearOfConst.cpp
  src/gui/dialogYearOfDemol.cpp
  src/gui/mainWindow.cpp
  src/gui/plugindialog.cpp
  src/gui/treeView.cpp
  
  src/gui/osg/osgAssimp.cpp
  src/gui/osg/osgCityGML.cpp
  src/gui/osg/osgCompass.cpp
  src/gui/osg/osgGDAL.cpp
  src/gui/osg/osgInfoBubbles.cpp
  src/gui/osg/osgLas.cpp
  src/gui/osg/osgPicking.cpp
  src/gui/osg/osgQtWidget.cpp
  src/gui/osg/osgScene.cpp
  src/gui/osg/osgTools.cpp
  
### Filters
  # src/libfilters/ChangeDetection/ChangeDetection.cpp
  # src/libfilters/EnhanceCityGML/EnhanceMNT.cpp
  # src/libfilters/EnhanceCityGML/LinkCityGMLShape.cpp
  # src/libfilters/raytracing/Hit.cpp
  # src/libfilters/raytracing/Ray.cpp
  # src/libfilters/raytracing/RayTracing.cpp
  # src/libfilters/ShpExtrusion/ShpExtrusion.cpp
  # src/libfilters/tiling/ASCCut.cpp
  # src/libfilters/tiling/TilingCityGML.cpp
  
### Now part of "Core" library
  # FIXME src/DataStructures/DEM/osgMnt.cpp
  # FIXME src/utils/OGRGDAL_Utils/OGRGDALtools.cpp
  # FIXME src/utils/OGRGDAL_Utils/OGRGDALtoShpWriter.cpp

### Now part of "citygmlutils" library
  # FIXME src/core/tile.cpp MOVE THE FILE THERE
  
### Utilities
  # Depends on citygml/exportCityGML.hpp
  src/utils/CityGMLFusion.cpp

  src/main.cpp

### FIXME section
  # FIXME: Interestingly enough the following are not necessary !?
  # src/gui/osgQt/osg32/GraphicsWindowQt.cxx
  # src/gui/osgQt/osg34/GraphicsWindowQt.cxx
  # src/plugins/*.cpp
)

SET( VCITY_GUI_SRC
  ${VCITY_GUI_SRC}
  ${VCITY_GUI_Qt_SRC}
  ${VCITY_GUI_UI_CPP}
  ${VCITY_GUI_RES_CPP}
)

# FIXME: the following section deals with OSG-QT and should probably
# be placed next to the find_package on OSGQT (refer above) in order
# to clarify the flow
if( WIN32 )
  # FIXME: the supported kit versions are KIT_VS2015 and KIT_VS2012
  # Use KIT_VS2012 instead of "NOT DEFINED KIT_VS2015"
  if(NOT DEFINED KIT_VS2015)
    SET( VCITY_GUI_SRC_WIN src/gui/osgQt/osg32/GraphicsWindowQt.cxx )
  endif()
  if(DEFINED KIT_VS2015)
    SET( VCITY_GUI_SRC_WIN src/gui/osgQt/osg34/GraphicsWindowQt.cxx )
  endif()

  SET( VCITY_GUI_SRC ${VCITY_GUI_SRC} ${VCITY_GUI_SRC_WIN} )
endif()
if(DEFINED ENV{TRAVIS})
  if (APPLE)
    set( VCITY_GUI_SRC_WIN src/gui/osgQt/osg34/GraphicsWindowQt.cxx )
    set( VCITY_GUI_SRC ${VCITY_GUI_SRC} ${VCITY_GUI_SRC_WIN} )
  endif (APPLE)
endif()

if (WITH_QT5)
else(WITH_QT5)
  SET( VCITY_GUI_SRC ${VCITY_GUI_SRC} ${VCITY_GUI_MOC_CPP} )
endif(WITH_QT5)

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
  ${LASLIB_LIBRARY}
  ${OPENGL_LIBRARIES}
)

if (WITH_QT5)
else(WITH_QT5)
  SET( VCITY_GUI_LIB ${VCITY_GUI_LIB} ${QT_LIBRARIES} )
endif(WITH_QT5)

set(my_COMPILE_DEFINITIONS "")
if (WITH_QT5)
  set(my_COMPILE_DEFINITIONS ${my_COMPILE_DEFINITIONS} ";WITH_QT5")
endif(WITH_QT5)

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

if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
  set(LINUX_FLAGS -lpthread)   # FIXME: assert this is needed
endif()

add_executable(${PRJ_NAME} MACOSX_BUNDLE ${VCITY_GUI_SRC} ${VCITY_GUI_ICON})

target_include_directories( ${PRJ_NAME} PUBLIC
  ${CITYGML_INCLUDE_DIR}
  ${FILTERS_INCLUDE_DIR} 
)

target_link_libraries( 3DUSE
  ${LINUX_FLAGS}  # FIXME: are these flags or a library !?
  ${VCITY_GUI_LIB}
  ${VCITYCORE_LIBRARY}
  ${FILTERS_LIBRARIES}
  ${CITYGML_LIBRARIES}
  ${CITYGMLUTILS_LIBRARIES}
)

if (WITH_QT5)
  # FIXME: refer to src/CMakeLists.txt to get rid of QT5_USE_MODULES
  add_definitions( "-DWITH_QT5" )
  QT5_USE_MODULES( 3DUSE Widgets Xml OpenGL )
endif(WITH_QT5)
SET_TARGET_PROPERTIES(${PRJ_NAME} PROPERTIES COMPILE_DEFINITIONS "${my_COMPILE_DEFINITIONS}")

add_executable( CityGMLCut
  src/utils/cmdline/cityGMLCut.cxx
  src/utils/cmdline/triangulate.cxx
)
target_include_directories( CityGMLCut PUBLIC src/libcitygml ) # For vecs.hpp
target_link_libraries(CityGMLCut ${VCITY_GUI_LIB})

if (WITH_QT5)
  ADD_DEFINITIONS("-DWITH_QT5")
  QT5_USE_MODULES( CityGMLCut Widgets Xml OpenGL )
endif(WITH_QT5)
SET_TARGET_PROPERTIES(CityGMLCut PROPERTIES COMPILE_DEFINITIONS "${my_COMPILE_DEFINITIONS}")

add_subdirectory(src)

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
	set( CityGMLCut_Qt_Plugin_HEADER
    src/plugins/CityGMLCutQtPlugin/CityGMLCutQtPlugin.h
    src/plugins/CityGMLCutQtPlugin/Dialogs/DialogCityGMLCut1.h)
	set( CityGMLCut_Qt_Plugin_UI
     src/plugins/CityGMLCutQtPlugin/Dialogs/DialogCityGMLCut1.ui)
	if (WITH_QT5)
	  ADD_DEFINITIONS("-DWITH_QT5")
	  QT5_WRAP_CPP( CityGMLCut_Qt_Plugin_MOC_CPP
      ${CityGMLCut_Qt_Plugin_HEADER}
      TARGET CityGMLCutQtPlugin
    )
	  QT5_WRAP_UI(CityGMLCut_Qt_Plugin_UI_CPP ${CityGMLCut_Qt_Plugin_UI})
	  set( CityGMLCut_Qt_Plugin_SRC
      ${CityGMLCut_Qt_Plugin_SRC}
      ${CityGMLCut_Qt_Plugin_MOC_CPP}
      ${CityGMLCut_Qt_Plugin_UI_CPP}
    )
	else(WITH_QT5) 
	  QT4_WRAP_CPP( CityGMLCut_Qt_Plugin_MOC_CPP
      ${CityGMLCut_Qt_Plugin_HEADER}
      TARGET CityGMLCutQtPlugin
    )
	  QT4_WRAP_UI(CityGMLCut_Qt_Plugin_UI_CPP ${CityGMLCut_Qt_Plugin_UI})
	  set(CityGMLCut_Qt_Plugin_SRC ${CityGMLCut_Qt_Plugin_SRC} ${CityGMLCut_Qt_Plugin_MOC_CPP} ${CityGMLCut_Qt_Plugin_UI_CPP})
	  set(CityGMLCut_Qt_Plugin_LIB ${CityGMLCut_Qt_Plugin_LIB} ${QT_LIBRARIES})
	endif(WITH_QT5)

  add_library( CityGMLCutQtPlugin SHARED
    src/plugins/CityGMLCutQtPlugin/CityGMLCutQtPlugin.cxx
    src/plugins/CityGMLCutQtPlugin/Dialogs/DialogCityGMLCut1.cxx
    src/utils/cmdline/cityGMLCut.cxx src/utils/cmdline/triangulate.cxx
    ${CityGMLCut_Qt_Plugin_HEADER}
    ${CityGMLCut_Qt_Plugin_SRC}
    ${VCITY_GUI_SRC_WIN}
  )
  target_include_directories( CityGMLCutQtPlugin PUBLIC
    src                            # For moc on Generic_PluginInterface
    ${CITYGML_INCLUDE_DIR}
  )
  target_link_libraries( CityGMLCutQtPlugin
    ${CityGMLCut_Qt_Plugin_LIB}
    ${VCITY_GUI_LIB}
  )
	if (WITH_QT5)
	  QT5_USE_MODULES(CityGMLCutQtPlugin Core Widgets OpenGL)
	endif(WITH_QT5)
endif (BUILD_CityGMLCutQtPlugin)
# --> CityGMLCutQtPlugin : QtPlugin [END]

# --> CityGMLEmptyQtPlugin : QtPlugin [BEGIN]
OPTION(BUILD_CityGMLEmptyQtPlugin "BUILD CityGMLEmptyQtPlugin " ON)
if (BUILD_CityGMLEmptyQtPlugin)
	set( CityGMLEmpty_Qt_Plugin_HEADER
    src/plugins/CityGMLEmptyQtPlugin/CityGMLEmptyQtPlugin.h
    src/plugins/CityGMLEmptyQtPlugin/Dialogs/DialogCityGMLEmpty1.h
  )
	set( CityGMLEmpty_Qt_Plugin_UI
    src/plugins/CityGMLEmptyQtPlugin/Dialogs/DialogCityGMLEmpty1.ui
  )
	if (WITH_QT5)
	  ADD_DEFINITIONS("-DWITH_QT5")
	  QT5_WRAP_CPP( CityGMLEmpty_Qt_Plugin_MOC_CPP
      ${CityGMLEmpty_Qt_Plugin_HEADER}
      TARGET CityGMLEmptyQtPlugin
    )
	  QT5_WRAP_UI(CityGMLEmpty_Qt_Plugin_UI_CPP ${CityGMLEmpty_Qt_Plugin_UI})
	  set(CityGMLEmpty_Qt_Plugin_SRC ${CityGMLEmpty_Qt_Plugin_SRC} ${CityGMLEmpty_Qt_Plugin_MOC_CPP} ${CityGMLEmpty_Qt_Plugin_UI_CPP})
	else(WITH_QT5) 
	  QT4_WRAP_CPP(CityGMLEmpty_Qt_Plugin_MOC_CPP ${CityGMLEmpty_Qt_Plugin_HEADER})
	  QT4_WRAP_UI(CityGMLEmpty_Qt_Plugin_UI_CPP ${CityGMLEmpty_Qt_Plugin_UI})
	  set(CityGMLEmpty_Qt_Plugin_SRC ${CityGMLEmpty_Qt_Plugin_SRC} ${CityGMLEmpty_Qt_Plugin_MOC_CPP} ${CityGMLEmpty_Qt_Plugin_UI_CPP})
	  set(CityGMLEmpty_Qt_Plugin_LIB ${CityGMLEmpty_Qt_Plugin_LIB} ${QT_LIBRARIES})
	endif(WITH_QT5)

  add_library( CityGMLEmptyQtPlugin SHARED
    src/plugins/CityGMLEmptyQtPlugin/CityGMLEmptyQtPlugin.cxx
    src/plugins/CityGMLEmptyQtPlugin/Dialogs/DialogCityGMLEmpty1.cxx
    ${CityGMLEmpty_Qt_Plugin_HEADER}
    ${CityGMLEmpty_Qt_Plugin_SRC}
    ${VCITY_GUI_SRC_WIN}
  )
  target_include_directories( CityGMLEmptyQtPlugin PUBLIC
    src                            # For moc on Generic_PluginInterface
    ${CITYGML_INCLUDE_DIR}
  )
  target_link_libraries( CityGMLEmptyQtPlugin
    ${CityGMLEmpty_Qt_Plugin_LIB}
    ${VCITY_GUI_LIB}
  )
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
  set( CityGMLVisibilite_Qt_Plugin_HEADER
    src/plugins/CityGMLVisibiliteQtPlugin/CityGMLVisibiliteQtPlugin.h
    src/plugins/CityGMLVisibiliteQtPlugin/Dialogs/dialogVisibilite.hpp
  )
	set( CityGMLVisibilite_Qt_Plugin_UI
    src/plugins/CityGMLVisibiliteQtPlugin/Dialogs/dialogVisibilite.ui
  )
	if (WITH_QT5)
	  ADD_DEFINITIONS("-DWITH_QT5")
	  QT5_WRAP_CPP( CityGMLVisibilite_Qt_Plugin_MOC_CPP
      ${CityGMLVisibilite_Qt_Plugin_HEADER}
      TARGET CityGMLVisibiliteQtPlugin
    )
	  QT5_WRAP_UI( CityGMLVisibilite_Qt_Plugin_UI_CPP
      ${CityGMLVisibilite_Qt_Plugin_UI}
    )
	  set( CityGMLVisibilite_Qt_Plugin_SRC
      ${CityGMLVisibilite_Qt_Plugin_SRC}
      ${CityGMLVisibilite_Qt_Plugin_MOC_CPP}
      ${CityGMLVisibilite_Qt_Plugin_UI_CPP}
    )
	else(WITH_QT5) 
	  QT4_WRAP_CPP(CityGMLVisibilite_Qt_Plugin_MOC_CPP ${CityGMLVisibilite_Qt_Plugin_HEADER})
	  QT4_WRAP_UI(CityGMLVisibilite_Qt_Plugin_UI_CPP ${CityGMLVisibilite_Qt_Plugin_UI})
	  set(CityGMLVisibilite_Qt_Plugin_SRC ${CityGMLVisibilite_Qt_Plugin_SRC} ${CityGMLVisibilite_Qt_Plugin_MOC_CPP} ${CityGMLVisibilite_Qt_Plugin_UI_CPP})
	  set(CityGMLVisibilite_Qt_Plugin_LIB ${CityGMLVisibilite_Qt_Plugin_LIB} ${QT_LIBRARIES})
	endif(WITH_QT5)

  add_library( CityGMLVisibiliteQtPlugin SHARED
     src/plugins/CityGMLVisibiliteQtPlugin/CityGMLVisibiliteQtPlugin.cxx
     src/plugins/CityGMLVisibiliteQtPlugin/Dialogs/dialogVisibilite.cxx
     ${CityGMLVisibilite_Qt_Plugin_HEADER}
     ${CityGMLVisibilite_Qt_Plugin_SRC}
     ${VCITY_GUI_SRC} # if need, add cxx files here before the ')'
  )
  # For raytracing:
  target_include_directories( CityGMLVisibiliteQtPlugin PUBLIC
    src                            # For moc on Generic_PluginInterface
    src/libfilters
    ${CITYGML_INCLUDE_DIR}
  )
  target_link_libraries( CityGMLVisibiliteQtPlugin
    ${LINUX_FLAGS}
    ${CITYGML_LIBRARIES}
    ${CITYGMLUTILS_LIBRARIES}
    ${FILTERS_LIBRARIES}
    ${CityGMLVisibilite_Qt_Plugin_LIB}
    ${VCITY_GUI_LIB}
  )

	if (WITH_QT5)
	  QT5_USE_MODULES(CityGMLVisibiliteQtPlugin Core Widgets OpenGL)
	endif(WITH_QT5)

endif (BUILD_CityGMLVisibiliteQtPlugin)
# --> CityGMLVisibiliteQtPlugin : QtPlugin [END]

# --> CityGMLFloodARQtPlugin : QtPlugin [BEGIN]
OPTION(BUILD_CityGMLFloodARQtPlugin "BUILD CityGMLFloodARQtPlugin " OFF)
if (BUILD_CityGMLFloodARQtPlugin) 
  set( CityGMLFloodAR_Qt_Plugin_HEADER
    src/plugins/CityGMLFloodARQtPlugin/CityGMLFloodARQtPlugin.h
    src/plugins/CityGMLFloodARQtPlugin/Dialogs/dialogFloodAR.hpp
    src/plugins/CityGMLFloodARQtPlugin/FloodARTools.hpp
  )
  set( CityGMLFloodAR_Qt_Plugin_UI
    src/plugins/CityGMLFloodARQtPlugin/Dialogs/dialogFloodAR.ui
  )
  if (WITH_QT5)
    ADD_DEFINITIONS("-DWITH_QT5")
    QT5_WRAP_CPP( CityGMLFloodAR_Qt_Plugin_MOC_CPP
      ${CityGMLFloodAR_Qt_Plugin_HEADER}
      TARGET CityGMLFloodARQtPlugin
    )
    QT5_WRAP_UI( CityGMLFloodAR_Qt_Plugin_UI_CPP
      ${CityGMLFloodAR_Qt_Plugin_UI}
    )
    set( CityGMLFloodAR_Qt_Plugin_SRC
      ${CityGMLFloodAR_Qt_Plugin_SRC}
      ${CityGMLFloodAR_Qt_Plugin_MOC_CPP}
      ${CityGMLFloodAR_Qt_Plugin_UI_CPP})
  else(WITH_QT5) 
    QT4_WRAP_CPP(CityGMLFloodAR_Qt_Plugin_MOC_CPP ${CityGMLFloodAR_Qt_Plugin_HEADER})
    QT4_WRAP_UI(CityGMLFloodAR_Qt_Plugin_UI_CPP ${CityGMLFloodAR_Qt_Plugin_UI})
    set(CityGMLFloodAR_Qt_Plugin_SRC ${CityGMLFloodAR_Qt_Plugin_SRC} ${CityGMLFloodAR_Qt_Plugin_MOC_CPP} ${CityGMLFloodAR_Qt_Plugin_UI_CPP})
    set(CityGMLFloodAR_Qt_Plugin_LIB ${CityGMLFloodAR_Qt_Plugin_LIB} ${QT_LIBRARIES})
  endif(WITH_QT5)

  add_library( CityGMLFloodARQtPlugin SHARED
    src/plugins/CityGMLFloodARQtPlugin/CityGMLFloodARQtPlugin.cxx
    src/plugins/CityGMLFloodARQtPlugin/Dialogs/dialogFloodAR.cxx
    src/plugins/CityGMLFloodARQtPlugin/FloodARTools.cxx
    ${CityGMLFloodAR_Qt_Plugin_HEADER}
    ${CityGMLFloodAR_Qt_Plugin_SRC}
    ${VCITY_GUI_SRC}
    # if needed, add cxx files here
  )

  target_include_directories( CityGMLFloodARQtPlugin PUBLIC
    src                            # For moc on Generic_PluginInterface
    ${FILTERS_INCLUDE_DIR}
    ${CITYGMLUTILS_INCLUDE_DIR}
  )

  target_link_libraries ( CityGMLFloodARQtPlugin
    ${LINUX_FLAGS}
    ${CityGMLFloodAR_Qt_Plugin_LIB}
    ${VCITY_GUI_LIB}
    ${CITYGML_LIBRARIES}
    ${CITYGMLUTILS_LIBRARIES}
    ${FILTERS_LIBRARIES}
  )
  if (WITH_QT5)
    QT5_USE_MODULES(CityGMLFloodARQtPlugin Core Widgets OpenGL)
  endif(WITH_QT5)
endif (BUILD_CityGMLFloodARQtPlugin)
# --> CityGMLFloodARQtPlugin : QtPlugin [END]

################
# END of plugins
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
if (BUILD_CityGMLFloodARQtPlugin)
    set(CityGMLFloodARQtPlugin "CityGMLFloodARQtPlugin")
endif (BUILD_CityGMLFloodARQtPlugin)
################
# 3DUSE plugins
################

INSTALL(TARGETS ${PRJ_NAME}
  CityGMLCut
  ${CityGMLCutQtPlugin}
  ${CityGMLEmptyQtPlugin}
  ${CityGMLVisibiliteQtPlugin}
  ${CityGMLFloodARQtPlugin}
  # add your plugin installation here
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

