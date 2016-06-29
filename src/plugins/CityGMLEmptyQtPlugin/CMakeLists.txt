if( NOT BUILD_CityGMLEmptyQtPlugin )
  return()
endif()

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

add_library( CityGMLEmptyQtPlugin SHARED
  CityGMLEmptyQtPlugin.cxx
  ${EMBARKED_OSGQT_SRC}
  Dialogs/DialogCityGMLEmpty1.cxx
  ${CityGMLEmpty_Qt_Plugin_HEADER}   # FIXME: adding header files looks pointless
  ${CityGMLEmpty_Qt_Plugin_SRC}
)

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

target_include_directories( CityGMLEmptyQtPlugin PUBLIC
  ${CMAKE_SOURCE_DIR}/src           # For moc on Generic_PluginInterface
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CITYGML_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

target_link_libraries( CityGMLEmptyQtPlugin
  # VCityGui          # FIXME: clean me up
  ${VCITY_GUI_LIB}
  ${QT_LIBRARIES}
)

if( BUILD_GUI_QT5 )
  target_include_directories( CityGMLEmptyQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLEmptyQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

# FIXME: temporarily disabled because of AppVeyor build crash
# install( TARGETS CityGMLEmptyQtPlugin
#   LIBRARY DESTINATION lib   # FIXME: inquire on this
#     COMPONENT ${PRJ_NAME}     # FIXME: is this line needed ?
#)
