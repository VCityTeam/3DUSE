project( 3DUSE )

cmake_minimum_required( VERSION 2.8.12 )
include( cmake/3DusePolicies.cmake )
include( GenerateExportHeader )  # For dll export header of VC++ shared lib

########### Building options:
option( BUILD_GUI_QT4 "Build the Graphical User Interface with QT4" ON )
option( BUILD_GUI_QT5 "Build the Graphical User Interface with QT5" OFF )

if( BUILD_GUI_QT4 AND BUILD_GUI_QT5 )
  # In case of simultaneous demand, and since Qt4 is ON by default, asking
  # for Qt5 simply means that the builder didn't turn Qt4 OFF after requiring
  # for Qt5 to be ON. Hence, as an historical convenience (refer to Appveyor 
  # scripts), when both Qt4 and Qt5 are one, Qt5 has preference overs Qt4:
  set( BUILD_GUI_QT4 OFF  CACHE BOOL
      "Build the Graphical User Interface with QT4."
      FORCE)
endif()

option( BUILD_EMBARKED_OSG-QT_32 "Use embarked copy of OSG-Qt version 3.2" OFF )
option( BUILD_EMBARKED_OSG-QT_34 "Use embarked copy of OSG-Qt version 3.4" OFF )

option( BUILD_DOCUMENTATION  "Build the documentation." OFF )

# Some syntactic convenience for CI scripts shortening:
option( BUILD_ALL_PLUGINS "Build all available plugins" OFF )
if( BUILD_ALL_PLUGINS )
  set( BUILD_CityGMLCutQtPlugin        ON CACHE BOOL
      "BUILD the Cut GUI plugin"       FORCE
  )
  set( BUILD_CityGMLEmptyQtPlugin      ON CACHE BOOL
      "BUILD the Empty GUI plugin"     FORCE
  )
  set( BUILD_CityGMLVisibiliteQtPlugin ON CACHE BOOL
      "Build the Visibility plugin."   FORCE
  )
  set( BUILD_CityGMLFloodARQtPlugin    ON CACHE BOOL
      "Build the FloodAR GUI plugin."  FORCE
  )
  set( BUILD_CityGMLSunlightQtPlugin   ON CACHE BOOL
      "Build the Sunlight GUI plugin." FORCE
  )
else()
  option( BUILD_CityGMLCutQtPlugin      "Build the Cut GUI plugin."      ON )
  option( BUILD_CityGMLEmptyQtPlugin    "Build the Empty GUI plugin."    ON )
  option( BUILD_CityGMLFloodARQtPlugin  "Build the FloodAR GUI plugin."  OFF)
  option( BUILD_CityGMLSunlightQtPlugin "Build the Sunlight GUI plugin." OFF)
  option( BUILD_CityGMLVisibiliteQtPlugin "Build the Visibilite plugin." OFF)
endif()


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
if( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" )
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
  # FIXME: Alas this breaks ctest (on MSVC) that is unable to find the
  # executables that were declared with add_test()...
  # foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
  #   string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
  #   set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
  #        ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG} )
  #   set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
  #        ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG} )
  #   set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
  #        ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG} )
  # endforeach()
endif()

##### OSX specifics (Apple)
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON) # see cmake POLICY CMP0042
endif(APPLE)

############################## Package (dependencies) detection
##### Find Qt5
if( BUILD_GUI_QT5 )
  # FIXME: Refer to comments in .travis.yml and appveyor.yml but this can
  # probably be cleaned up.
  if(DEFINED ENV{QT5_DIR})
    set( QT5_DIR $ENV{QT5_DIR} )
  endif()
  SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${QT5_DIR})

  # FIXME: remove those nested and make it flat !
  FIND_PACKAGE( Qt5Core )
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
        endif()
      else()
        message(FATAL_ERROR "Qt5Xml not found.")
      endif()
    else()
      message(FATAL_ERROR "Qt5Widgets not found.")
    endif()
  else()
    message(FATAL_ERROR "Qt5Core not found. Please set QT5_DIR.")
  endif()
endif()

##### Find Qt4 
if( BUILD_GUI_QT4 )
  # FIXME: note that the "corresponding"
  #     add_definitions( "-DBUILD_GUI_QT5" )
  # is located elsewhere which breaks the QT4 vs QT5 "symmetry". Why is this ?
  add_definitions( "-DBUILD_GUI_QT4" )
  # FIXME: ask MTO but this looks like the opposite of what Qt5 does above
  if (QTDIR)
    set( ENV{QTDIR} ${QTDIR} )
  endif()
  set( QT_USE_QTMAIN   TRUE )
  set( QT_USE_QTXML    TRUE )
  set( QT_USE_QTOPENGL TRUE )
  find_package( Qt4 REQUIRED )
  include( ${QT_USE_FILE} )
endif()

# Find OpenGL
# FIXME: the only dependency towards OpenGL occurs 
# src/libcitygml/tesselator.hpp which includes glu.h. Deport this
# dependency to libCityGML (and even better try to remove it completely
# by using self contained code or CGAL tesselators).
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


# Find GDAL
find_package(GDAL)
if(GDAL_FOUND)
  # FIXME: remove this line and (if needed) use target_include_directories...
  include_directories(${GDAL_INCLUDE_DIR})
else(GDAL_FOUND)
  message(FATAL_ERROR "GDAL not found. Please set GDAL_DIR or GDAL_ROOT.")
endif(GDAL_FOUND)

# Find LibXml2
# FIXME: as discovered with the command
#    grep -r "#include" * | grep -i libxml
# LibXml2 is used only within libCityGML (which is a fair usage) and
# src/utils/cmdline/cityGMLCut.h which (besides from the command line
# src/utils/cmdline/cityGMLCut.cxx) gets strangely included in 
#     src/plugins/CityGMLCutQtPlugin/CityGMLCutQtPlugin.cxx.
# Assert with FDE that is not a real need and deport the remaining
# find_package to LibXml2 where it belongs i.e. in the libCityGML
# component.
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

# Find OpenThreads (which is also an OSG sub-module)
find_package(OpenThreads)
if(OPENTHREADS_FOUND)
  include_directories(${OPENTHREADS_INCLUDE_DIR})
else(OPENTHREADS_FOUND)
  message(FATAL_ERROR "OpenThreads not found. Please set OPENTHREADS_DIR.")
endif(OPENTHREADS_FOUND)

# Find osgQt
# 3DUse embarks its own "fork" of OSQ-Qt (within src/gui/osgQt sub-directory).
# Choosing whether to use the "native" version (i.e. the one found with
# find_package() or the embarked version is triggered manually with the
# USE_BUILT-IN_OSG-QT-XX (where XX stands for the version number) flags.
# When using the embarked version the additional source file names that
# need to be compiled are placed within the EMBARKED_OSGQT_SRC variable.
# Refer to doc/SoftwareEngineering/CMakeChoices.md for further design
# notes on this matter.
if( BUILD_EMBARKED_OSG-QT_32 )
  if( WIN32 )
    if( DEFINED KIT_VS2015 )
      message( WARNING "Trying to use OSG-Qt version 32 with Kit 2015 !?")
    endif()
    set( OSGQT_INCLUDE_DIR "" )
  else()
    # Apple, Linux...
    set( OSGQT_INCLUDE_DIR "src/gui/osgQt/osg32/include" )
  endif()
  # FIXME: remove systematic inclusion and use include_target_directories()
  include_directories( ${OSGQT_INCLUDE_DIR} )
  set( EMBARKED_OSGQT_SRC
       ${CMAKE_SOURCE_DIR}/src/gui/osgQt/osg32/GraphicsWindowQt.cxx
  )
elseif( BUILD_EMBARKED_OSG-QT_34 )
  if( WIN32 )
    if( NOT DEFINED KIT_VS2015 )
      message( WARNING "Trying to use OSG-Qt version 34 with Kit 2012 !?")
    endif()
    set( OSGQT_INCLUDE_DIR "" )
  else()
    # Apple, Linux...
    set( OSGQT_INCLUDE_DIR "src/gui/osgQt/osg34/include" )
  endif()
  # FIXME: remove systematic inclusion and use include_target_directories()
  include_directories( ${OSGQT_INCLUDE_DIR} )
  set( EMBARKED_OSGQT_SRC
       ${CMAKE_SOURCE_DIR}/src/gui/osgQt/osg34/GraphicsWindowQt.cxx
  )
else( )
  # Look for the "native" version:
  find_package( osgQt )
  if( OSGQT_FOUND )
    # FIXME remove and use target_include_directories instead...
    include_directories(${OSGQT_INCLUDE_DIR})
  else()
    message(FATAL_ERROR "osgQt not found. Either set OSG_DIR or use USE_BUILT-IN_OSG-QT_XX flags.")
  endif()
endif()

##############
# FIXME: expand the following obfuscating two glob_recurse
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

if( BUILD_GUI_QT4 )
  QT4_WRAP_CPP(      VCITY_GUI_MOC_CPP ${VCITY_GUI_Qt_SRC})
  QT4_WRAP_UI(       VCITY_GUI_UI_CPP  ${VCITY_GUI_Qt_UI})
  QT4_ADD_RESOURCES( VCITY_GUI_RES_CPP ${VCITY_GUI_Qt_RES})
endif()

if( BUILD_GUI_QT5 )
  # FIXME: Running automoc automatically seems to create undocumented
  # difficulties. Inquire on this matter and either document the encountered
  # difficulties or resume with the default mechanism.
  set( CMAKE_AUTOMOC OFF )
  QT5_WRAP_CPP(      VCITY_GUI_MOC_CPP ${VCITY_GUI_Qt_SRC} )
  QT5_WRAP_UI(       VCITY_GUI_UI_CPP  ${VCITY_GUI_Qt_UI})
  # The following line issues the CMake Warning (from Qt5CoreMacros.cmake:224):
  #    "configure_file called with unknown argument COPY_ONLY"
  # This is a qt bug (see reference below) that is fixed above QT 5.4.1.
  # Reference: https://bugreports.qt.io/browse/QTBUG-44637
  QT5_ADD_RESOURCES( VCITY_GUI_RES_CPP ${VCITY_GUI_Qt_RES})
endif()


##### Local libraries
# The following variable definitions add syntactic sugat in an attempt
# to unify notations between external libraries and internal (defined
# within this project) libraries. 
# FIXME: those definition should be deported to their respective
# sub-directories and defined as set( <variable> <value> CACHE INTERNAL )

# CORE FIXME: rename CORE TO Utils !
set( VCITYCORE_INCLUDE_DIR
  #${CMAKE_SOURCE_DIR}/src             # FIXME: this should be the last one !
  ${CMAKE_BINARY_DIR}/src              # for generated vcitycore_export.h
)
set( VCITYCORE_LIBRARY vcitycore )

# CITYGML--> CORE
set( CITYGML_INCLUDE_DIR
  ${CMAKE_SOURCE_DIR}/src              # FIXME: this should be the last one !
  ${CMAKE_SOURCE_DIR}/src/libcitygml   # FIXME: Used to find vecs.hpp
                                       # BTW move vecs.hpp to CORE
  ${CMAKE_BINARY_DIR}/src/libcitygml   # for generated citygml_export.h
)
set( CITYGML_LIBRARIES citygml )

# CITYGML UTILS--> CITYGML
set( CITYGMLUTILS_INCLUDE_DIR
  ${CMAKE_SOURCE_DIR}/src/libcitygml/utils
  ${CMAKE_BINARY_DIR}/src/libcitygml/utils  # generated citygmlutils_export.h
)
set( CITYGMLUTILS_LIBRARIES citygmlutils )

# FILTERS--> CORE, CITYGML, CITYGMLUTILS
set( FILTERS_INCLUDE_DIR
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_BINARY_DIR}/src/filters    # for  generated filters_export.h
)
set( FILTERS_LIBRARIES   filters )


# FIXME: the following line should be made away with. It is used by some
# plugins to find their includes. But this is because the plugins build
# commands are currently defined from within this central CMakeFile and
# thus with a path that is not relative to the code... When moving the
# plugin related directive to their respective directories, the following
# line can be removed
set(CMAKE_INCLUDE_CURRENT_DIR ON)

################### General
### Config files
configure_file( 
  src/gui/fileLayoutConfig.h.in
  ${CMAKE_BINARY_DIR}/src/gui/fileLayoutConfig.h
  @ONLY )

add_library( VCityGui STATIC
### The core
  src/core/RayBox.cpp
  src/core/abstractlayer.cpp
  src/core/algo.cpp
  src/core/algo2.cpp
  src/core/application.cpp
  src/core/cell.cpp
  src/core/controller.cpp
  src/core/dataprofile.cpp
  src/core/dateTime.cpp
  src/core/layerAssimp.cpp
  src/core/layerCityGML.cpp
  src/core/layerLas.cpp
  src/core/layerMnt.cpp
  src/core/layerShp.cpp
  src/core/layerInfo.cpp
  src/core/layerTiledCityGML.cpp
  src/core/scene.cpp
  src/core/settings.cpp
  src/core/tools/log.cpp
  src/core/search/criteria.cpp
  src/core/search/document.cpp
### The gui  
  src/gui/applicationGui.cpp
  src/gui/controllerGui.cpp
  src/gui/dialogAbout.cpp
  src/gui/dialogAddBuilding.cpp
  src/gui/dialogAddLayer.cpp
  src/gui/dialogBuildBuildingAABBs.cpp
  src/gui/dialogDoc.cpp
  src/gui/dialogEditAssimpNode.cpp
  src/gui/dialogEditBldg.cpp
  src/gui/dialogEditLayer.cpp
  src/gui/dialogEditTile.cpp
  src/gui/dialogLink.cpp
  src/gui/dialogLoadBBox.cpp
  src/gui/dialogSettings.cpp
  src/gui/dialogShpTool.cpp
  src/gui/dialogTilingCityGML.cpp
  src/gui/dialogConvertObjToCityGML.cpp
  src/gui/dialogYearOfConst.cpp
  src/gui/dialogYearOfDemol.cpp
  src/gui/mainWindow.cpp
  src/gui/plugindialog.cpp
  src/gui/treeView.cpp
  
  src/gui/osg/osgAssimp.cpp
  src/gui/osg/osgCityGML.cpp
  src/gui/osg/osgGDAL.cpp
  src/gui/osg/osgLas.cpp
  src/gui/osg/osgPicking.cpp
  src/gui/osg/osgQtWidget.cpp
  src/gui/osg/osgScene.cpp
  src/gui/osg/osgTools.cpp
  src/gui/osg/osgInfo.cpp
  src/gui/osg/osgInfoDataType.cpp
  src/gui/osg/osgUpdateInfo.cpp
  src/gui/osg/osgSkybox.cpp
  
### Utilities
  # Depends on citygml/exportCityGML.hpp
  src/utils/CityGMLFusion.cpp
  src/utils/SkylineComparison.cpp

### QT related:
  ${VCITY_GUI_Qt_SRC}
  ${VCITY_GUI_UI_CPP}
  ${VCITY_GUI_RES_CPP}
  ${VCITY_GUI_MOC_CPP}
  ${EMBARKED_OSGQT_SRC}       # Might be empty
)

# Embedding of a static library within the plugins (which must be dynamic
# libraries) requires the code to be agnostic in terms of position
# (PIC=Position-Independent Code).
set_target_properties( VCityGui PROPERTIES POSITION_INDEPENDENT_CODE ON )

target_include_directories( VCityGui PUBLIC
  ${VCITYCORE_INCLUDE_DIR}
  ${CITYGML_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
  ${FILTERS_INCLUDE_DIR}
  ${CMAKE_BINARY_DIR}/src/gui #For configFileLayout.h
)

if( BUILD_GUI_QT4 )
  # Strangely enough we are missing the ad-hoc target_include_directories() 
endif()

if( BUILD_GUI_QT5 )
  add_definitions( "-DBUILD_GUI_QT5" )   # FIXME: do we need this ?
  target_include_directories( VCityGui PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( VCityGui Qt5::Widgets Qt5::OpenGL )
endif()

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
  ${OPENTHREADS_LIBRARY}
  ${GDAL_LIBRARY}
  ${LIBXML2_LIBRARIES}
  ${LASLIB_LIBRARY}
  ${OPENGL_LIBRARIES}
)

if( BUILD_GUI_QT4 )
  set( VCITY_GUI_LIB ${VCITY_GUI_LIB} ${QT_LIBRARIES} )
endif()

# FIXME: get rid of this my_COMPILE_DEFINITIONS variable 
set(my_COMPILE_DEFINITIONS "")
if( BUILD_GUI_QT5 )
  set(my_COMPILE_DEFINITIONS ${my_COMPILE_DEFINITIONS} ";BUILD_GUI_QT5")
endif()

# --- EXECUTABLE ---

if( BUILD_GUI_QT5 )
  if( NOT DEFINED QT_PLUGINS_DIR )
    set( QT_PLUGINS_DIR "${QT5_DIR}/plugins" )
  endif( )
endif()

IF(WIN32)
 #------------------------------------------------------------------------------
 # Copy Qt plugins to 'Debug and Release' directories and configure qt.conf file
 #------------------------------------------------------------------------------
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

##### Executable for the application
set(APP_NAME 3DUSE)
# For Windows set the ico file with the rc file
IF(WIN32)
  SET(VCITY_GUI_ICON resources/vcity.rc)
ENDIF()

# Find Boost
if( APPLE )
  # Context: cmake 3.6.x, boost 1.60
  # Strangely enough boost if only found with a small caps leading b
  # (that is trying to find_package on Boost will fail)...
  find_package( boost COMPONENTS date_time REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS})
  set (Boost_LIBRARIES boost_date_time)
else()  # Linux and MSVC (mind you)
  find_package( Boost COMPONENTS date_time)
  if( NOT Boost_FOUND )
    message(WARNING "Boost cannot be found.")
  else ()
    include_directories(${Boost_INCLUDE_DIRS})
    if( UNIX )
      set (Boost_LIBRARIES boost_date_time)
    else()
      message(WARNING "aaaaaaaaaaaaaaaa WIN32 ")
      message(WARNING "bbbbbbbbbbbbbbbb Boost_LIBRARIES :" ${Boost_LIBRARIES})
   endif()
  endif ()
endif()

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

add_executable( ${APP_NAME} MACOSX_BUNDLE 
  src/main.cpp
  ${VCITY_GUI_ICON}
)

target_include_directories( ${APP_NAME} PUBLIC
  ${CITYGML_INCLUDE_DIR}   # FIXME: there is a mismatch between the included
  ${FILTERS_INCLUDE_DIR}   #        directories and the linked libraries.
)

target_link_libraries( ${APP_NAME}
  ${LINUX_FLAGS}  # FIXME: are these flags or a library !?
  VCityGui
  ${VCITY_GUI_LIB}
  ${VCITYCORE_LIBRARY}
  ${FILTERS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CITYGML_LIBRARIES}
  ${CITYGMLUTILS_LIBRARIES}
)

if( BUILD_GUI_QT5 )
  add_definitions( "-DBUILD_GUI_QT5" )   # FIXME: do we need this ?
  target_include_directories( ${APP_NAME} PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( ${APP_NAME} Qt5::Widgets Qt5::OpenGL )
endif()

if( NOT ( BUILD_EMBARKED_OSG-QT_32 OR BUILD_EMBARKED_OSG-QT_34 ) )
  target_link_libraries( ${APP_NAME} ${OSGQT_LIBRARIES} )
endif()

set_target_properties( ${APP_NAME} PROPERTIES
                       COMPILE_DEFINITIONS "${my_COMPILE_DEFINITIONS}"
)

##### CityGMLCut executable
add_executable( CityGMLCut
  src/utils/cmdline/cityGMLCut.cxx
  src/utils/cmdline/triangulate.cxx
)
target_include_directories( CityGMLCut PUBLIC src/libcitygml ) # For vecs.hpp
target_link_libraries(CityGMLCut ${VCITY_GUI_LIB})

if( BUILD_GUI_QT5 )
  ADD_DEFINITIONS("-DBUILD_GUI_QT5")
  target_include_directories( CityGMLCut PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLCut Qt5::Widgets )
endif()

SET_TARGET_PROPERTIES(CityGMLCut PROPERTIES COMPILE_DEFINITIONS "${my_COMPILE_DEFINITIONS}")

# The location on Recursion on src is only constrained to be below the
# definition of BUILD_CityGMLSunlightQtPlugin configuration symbols (refer
# to their usage in e.g. src/plugins/CMakeLists.txt. For the time being
# it is located down here only for historical reasons (plugin definitions
# used to be located in the above lines)...
add_subdirectory( src )
add_subdirectory( doc )

#-------------------------------------------------------------------------------
# Now the installation stuff :
# References:
#   - http://www.cmake.org/Wiki/BundleUtilitiesExample
#   - https://cst.version.fz-juelich.de/jupedsim/jpseditor/blob/b5bf1a7d2eec10cb07bcca7d64b76beaf970b024/CMakeLists.txt
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

INSTALL(TARGETS
  ${PRJ_NAME}
  CityGMLCut
  BUNDLE DESTINATION . COMPONENT ${PRJ_NAME}
  RUNTIME DESTINATION . COMPONENT ${PRJ_NAME}
  LIBRARY DESTINATION . COMPONENT ${PRJ_NAME}
)

#--------------------------------------------------------------------------------
# Install needed Qt plugins by copying directories from the qt installation
# One can cull what gets copied by using 'REGEX "..." EXCLUDE'
INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats" DESTINATION ${plugin_dest_dir}/plugins COMPONENT ${PRJ_NAME})
if( BUILD_GUI_QT5 )
  INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/platforms" DESTINATION ${plugin_dest_dir}/plugins COMPONENT ${PRJ_NAME})
endif()

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
		include(CPack)
ENDIF(WIN32)
#--------------------------------------------------------------------------------
# Now the installation stuff : end
#--------------------------------------------------------------------------------
