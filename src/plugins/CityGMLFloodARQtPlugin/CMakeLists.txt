if( NOT BUILD_CityGMLFloodARQtPlugin ) 
  return()
endif()

####### Collecting material for QT generated wrappers:
# FIXME: be coherent with the include file extensions
set( CityGMLFloodAR_Qt_Plugin_HEADER
  CityGMLFloodARQtPlugin.h
  Dialogs/dialogFloodAR.hpp
  FloodARTools.hpp
)
set( CityGMLFloodAR_Qt_Plugin_UI
  Dialogs/dialogFloodAR.ui
)
if( BUILD_GUI_QT5 )
  ADD_DEFINITIONS("-DBUILD_GUI_QT5")
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
endif() 
if( BUILD_GUI_QT4 )
  QT4_WRAP_CPP( CityGMLFloodAR_Qt_Plugin_MOC_CPP
    ${CityGMLFloodAR_Qt_Plugin_HEADER}
    TARGET CityGMLFloodARQtPlugin
  )
  QT4_WRAP_UI(CityGMLFloodAR_Qt_Plugin_UI_CPP ${CityGMLFloodAR_Qt_Plugin_UI})
  set(CityGMLFloodAR_Qt_Plugin_SRC
     ${CityGMLFloodAR_Qt_Plugin_SRC}
     ${CityGMLFloodAR_Qt_Plugin_MOC_CPP}
     ${CityGMLFloodAR_Qt_Plugin_UI_CPP}
  )
endif()

####### The library per se:
add_library( CityGMLFloodARQtPlugin SHARED
  ${CityGMLFloodAR_Qt_Plugin_HEADER}
  ${CityGMLFloodAR_Qt_Plugin_SRC}
  ${EMBARKED_OSGQT_SRC}
  CityGMLFloodARQtPlugin.cxx
  Dialogs/dialogFloodAR.cxx
  FloodARTools.cxx
)

####### Destination location:
if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set_target_properties( CityGMLFloodARQtPlugin PROPERTIES
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
  set_target_properties( CityGMLFloodARQtPlugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()

####### Preprocessing paths:
target_include_directories( CityGMLFloodARQtPlugin PUBLIC
  src                            # For moc on Generic_PluginInterface
  ${FILTERS_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

####### Linking dependencies:
target_link_libraries ( CityGMLFloodARQtPlugin
  ${LINUX_FLAGS}        #FIXME: defined as -lpthread, WTF ?
  ${VCITY_GUI_LIB}
  ${CITYGML_LIBRARIES}
  ${CITYGMLUTILS_LIBRARIES}
  ${FILTERS_LIBRARIES}
)

if( BUILD_GUI_QT4 )
  target_link_libraries( CityGMLFloodARQtPlugin ${QT_LIBRARIES} )
endif()

if( BUILD_GUI_QT5 )
  target_include_directories( CityGMLFloodARQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLFloodARQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

if( NOT ( BUILD_EMBARKED_OSG-QT_32 OR BUILD_EMBARKED_OSG-QT_34 ) )
  target_link_libraries( CityGMLFloodARQtPlugin ${OSGQT_LIBRARIES} )
endif()
