add_subdirectory(Test)

if( NOT BUILD_CityGMLSunlightQtPlugin)
  return()
endif()

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

add_library( CityGMLSunlightQtPlugin SHARED
  ${CityGMLSunlight_Qt_Plugin_HEADER}
  ${CityGMLSunlight_Qt_Plugin_SRC}
  CityGMLSunlightQtPlugin.cxx
  Dialogs/DialogCityGMLSunlight1.cxx
  FileInfo.cxx
  IO.cxx
  SunlightDetection.cxx
)

target_include_directories( CityGMLSunlightQtPlugin PUBLIC
  src                            # For moc on Generic_PluginInterface
  ${FILTERS_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

target_link_libraries ( CityGMLSunlightQtPlugin
  VCityGui
  ${LINUX_FLAGS}
  ${QT_LIBRARIES}
  ${VCITY_GUI_LIB}
  ${CITYGML_LIBRARIES}
  ${CITYGMLUTILS_LIBRARIES}
  ${FILTERS_LIBRARIES}
)
if( BUILD_GUI_QT5 )
  target_include_directories( CityGMLSunlightQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLSunlightQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

if( NOT ( BUILD_EMBARKED_OSG-QT_32 OR BUILD_EMBARKED_OSG-QT_34 ) )
  target_link_libraries( CityGMLSunlightQtPlugin ${OSGQT_LIBRARIES} )
endif()

install( TARGETS CityGMLSunlightQtPlugin
  LIBRARY DESTINATION . COMPONENT ${PRJ_NAME}  # FIXME: is this line needed ?
)
