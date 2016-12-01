if( NOT BUILD_CityGMLCutQtPlugin )
  return()
endif()

####### Collecting material for QT generated wrappers:
set( CityGMLCut_Qt_Plugin_HEADER
  CityGMLCutQtPlugin.h
  Dialogs/DialogCityGMLCut1.h)
set( CityGMLCut_Qt_Plugin_UI
   Dialogs/DialogCityGMLCut1.ui
)

if( BUILD_GUI_QT5 )
  ADD_DEFINITIONS("-DBUILD_GUI_QT5")
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
endif() 
if( BUILD_GUI_QT4 )
  QT4_WRAP_CPP( CityGMLCut_Qt_Plugin_MOC_CPP
    ${CityGMLCut_Qt_Plugin_HEADER}
    TARGET CityGMLCutQtPlugin
  )
  QT4_WRAP_UI( CityGMLCut_Qt_Plugin_UI_CPP ${CityGMLCut_Qt_Plugin_UI} )
  set( CityGMLCut_Qt_Plugin_SRC
    ${CityGMLCut_Qt_Plugin_SRC}
    ${CityGMLCut_Qt_Plugin_MOC_CPP}
    ${CityGMLCut_Qt_Plugin_UI_CPP}
  )
endif()

####### The library per se:
add_library( CityGMLCutQtPlugin SHARED
  ${CityGMLCut_Qt_Plugin_HEADER}
  ${CityGMLCut_Qt_Plugin_SRC}
  ${EMBARKED_OSGQT_SRC}
  CityGMLCutQtPlugin.cxx
  Dialogs/DialogCityGMLCut1.cxx
  #FIXME: why does this plugin depend on cmdline (both following lines)?
  # There should treatment/process code (independent from QT) on which
  # depend both the QT plugin and the Command Line Inteface (CLI). But having
  # the CLI depend on QT is awkward !
  ${CMAKE_SOURCE_DIR}/src/utils/cmdline/cityGMLCut.cxx
  ${CMAKE_SOURCE_DIR}/src/utils/cmdline/triangulate.cxx
)

####### Destination location:
if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    # FIXME: since the plugin is a library the RUNTIME_OUTPUT_DIRECTORY_*
    # entry looks like not necessary
    set_target_properties( CityGMLCutQtPlugin PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}
      LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/Lib
    )
  endforeach()
endif()

if(UNIX)
  # A QT main applications looks for possible plugins in the application's
  # executable directory (among different conventional places). Note that,
  # strangely enough, on OSX (and as opposed to linux) the application ends
  # up being built in the plugin_dest_dir directory (refer to main
  # CMakeList.txt). Yet placing the plugin within ${CMAKE_BINARY_DIR}
  # still seems to do the trick:
  set_target_properties( CityGMLCutQtPlugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()

# FIXME: temporarily disabled because of AppVeyor build crash
# install( TARGETS CityGMLCutQtPlugin
#   LIBRARY DESTINATION . COMPONENT ${PRJ_NAME}
# )

####### Preprocessing paths:
target_include_directories( CityGMLCutQtPlugin PUBLIC
  ${CMAKE_SOURCE_DIR}/src/gui              # For moc on Generic_PluginInterface
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CITYGML_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

####### Linking dependencies:
if( BUILD_GUI_QT4 )
  target_link_libraries( CityGMLCutQtPlugin ${QT_LIBRARIES} )
endif()
if( BUILD_GUI_QT5 )
  target_include_directories( CityGMLCutQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLCutQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

target_link_libraries( CityGMLCutQtPlugin ${VCITY_GUI_LIB} )
