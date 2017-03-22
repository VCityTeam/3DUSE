add_subdirectory(Test)

if( NOT BUILD_CityGMLSunlightQtPlugin)
  return()
endif()

####### Collecting material for QT generated wrappers:
set( CityGMLSunlight_Qt_Plugin_HEADER
  CityGMLSunlightQtPlugin.h
  Dialogs/DialogCityGMLSunlight1.h
)
set( CityGMLSunlight_Qt_Plugin_UI
  Dialogs/DialogCityGMLSunlight1.ui
)

if ( BUILD_GUI_QT5 )
  ADD_DEFINITIONS("-DBUILD_GUI_QT5")
  QT5_WRAP_CPP( CityGMLSunlight_Qt_Plugin_MOC_CPP
    ${CityGMLSunlight_Qt_Plugin_HEADER}
    TARGET CityGMLSunlightQtPlugin
  )
  QT5_WRAP_UI( CityGMLSunlight_Qt_Plugin_UI_CPP
    ${CityGMLSunlight_Qt_Plugin_UI}
  )
  set( CityGMLSunlight_Qt_Plugin_SRC
    ${CityGMLSunlight_Qt_Plugin_SRC}
    ${CityGMLSunlight_Qt_Plugin_MOC_CPP}
    ${CityGMLSunlight_Qt_Plugin_UI_CPP}
  )
endif()

if( BUILD_GUI_QT4 )
  QT4_WRAP_CPP( CityGMLSunlight_Qt_Plugin_MOC_CPP
    ${CityGMLSunlight_Qt_Plugin_HEADER}
    TARGET CityGMLSunlightQtPlugin
  )
  QT4_WRAP_UI( CityGMLSunlight_Qt_Plugin_UI_CPP
    ${CityGMLSunlight_Qt_Plugin_UI}
  )
  set( CityGMLSunlight_Qt_Plugin_SRC
    ${CityGMLSunlight_Qt_Plugin_SRC}
    ${CityGMLSunlight_Qt_Plugin_MOC_CPP}
    ${CityGMLSunlight_Qt_Plugin_UI_CPP}
  )
endif()

####### The library per se:
add_library( CityGMLSunlightQtPlugin SHARED
  ${CityGMLSunlight_Qt_Plugin_HEADER}
  ${CityGMLSunlight_Qt_Plugin_SRC}
  CityGMLSunlightQtPlugin.cxx
  Dialogs/DialogCityGMLSunlight1.cxx
  FileInfo.cxx
  IO.cxx
  SunlightDetection.cxx
)

####### Destination location
if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set_target_properties( CityGMLSunlightQtPlugin PROPERTIES
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
  set_target_properties( CityGMLSunlightQtPlugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()

####### Preprocessing paths:
target_include_directories( CityGMLSunlightQtPlugin PUBLIC
  src                            # For moc on Generic_PluginInterface
  ${FILTERS_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

####### Linking dependencies:
target_link_libraries ( CityGMLSunlightQtPlugin
  VCityGui
  ${LINUX_FLAGS}     # FIXME: defined as -lpthread, WTF ?
  ${QT_LIBRARIES}
  ${VCITY_GUI_LIB}
  ${CITYGML_LIBRARIES}
  ${CITYGMLUTILS_LIBRARIES}
  ${FILTERS_LIBRARIES}
)

if( BUILD_GUI_QT5 )
  # FIXME: considering that there is no corresponding QT4 linking directive
  # probably that the following QT5 directives are not necessary. Or maybe
  # they are some hidden (global?) QT4 directives. Inquire on this...
  target_include_directories( CityGMLSunlightQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLSunlightQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

if( NOT ( BUILD_EMBARKED_OSG-QT_32 OR BUILD_EMBARKED_OSG-QT_34 ) )
  target_link_libraries( CityGMLSunlightQtPlugin ${OSGQT_LIBRARIES} )
endif()

####### Copy data file of plugin into binaries
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Data/AnnualSunPath_Lyon.csv ${CMAKE_CURRENT_BINARY_DIR}/Data/AnnualSunPath_Lyon.csv COPYONLY)
