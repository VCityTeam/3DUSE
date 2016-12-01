if( NOT BUILD_CityGMLEmptyQtPlugin )
  return()
endif()

####### Collecting material for QT generated wrappers:
set( CityGMLEmpty_Qt_Plugin_HEADER
  CityGMLEmptyQtPlugin.h
  Dialogs/DialogCityGMLEmpty1.h
)
set( CityGMLEmpty_Qt_Plugin_UI
  Dialogs/DialogCityGMLEmpty1.ui
)

if( BUILD_GUI_QT4 )
  QT4_WRAP_CPP( CityGMLEmpty_Qt_Plugin_MOC_CPP
    ${CityGMLEmpty_Qt_Plugin_HEADER}
    TARGET CityGMLEmptyQtPlugin
  )
  QT4_WRAP_UI( CityGMLEmpty_Qt_Plugin_UI_CPP ${CityGMLEmpty_Qt_Plugin_UI} )
  set( CityGMLEmpty_Qt_Plugin_SRC
    ${CityGMLEmpty_Qt_Plugin_SRC}
    ${CityGMLEmpty_Qt_Plugin_MOC_CPP}
    ${CityGMLEmpty_Qt_Plugin_UI_CPP}
  )
endif()

if( BUILD_GUI_QT5 )
  ADD_DEFINITIONS("-DBUILD_GUI_QT5")
  QT5_WRAP_CPP( CityGMLEmpty_Qt_Plugin_MOC_CPP
    ${CityGMLEmpty_Qt_Plugin_HEADER}
    TARGET CityGMLEmptyQtPlugin
  )
  QT5_WRAP_UI( CityGMLEmpty_Qt_Plugin_UI_CPP ${CityGMLEmpty_Qt_Plugin_UI} )
  set( CityGMLEmpty_Qt_Plugin_SRC
    ${CityGMLEmpty_Qt_Plugin_SRC}
    ${CityGMLEmpty_Qt_Plugin_MOC_CPP}
    ${CityGMLEmpty_Qt_Plugin_UI_CPP}
  )
endif() 

####### The library per se:
add_library( CityGMLEmptyQtPlugin SHARED
  CityGMLEmptyQtPlugin.cxx
  ${EMBARKED_OSGQT_SRC}
  Dialogs/DialogCityGMLEmpty1.cxx
  #${CityGMLEmpty_Qt_Plugin_HEADER}   # FIXME: adding header files looks pointless
  ${CityGMLEmpty_Qt_Plugin_SRC}
)

####### Destination location:
if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set_target_properties( CityGMLEmptyQtPlugin PROPERTIES
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
  set_target_properties( CityGMLEmptyQtPlugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()

####### Preprocessing paths:
target_include_directories( CityGMLEmptyQtPlugin PUBLIC
  ${CMAKE_SOURCE_DIR}/src           # For moc on Generic_PluginInterface
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CITYGML_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

####### Linking dependencies:
if( BUILD_GUI_QT4 )
  target_link_libraries( CityGMLEmptyQtPlugin ${QT_LIBRARIES} )
endif()

if( BUILD_GUI_QT5 )
  target_include_directories( CityGMLEmptyQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLEmptyQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

target_link_libraries( CityGMLEmptyQtPlugin ${VCITY_GUI_LIB} )
