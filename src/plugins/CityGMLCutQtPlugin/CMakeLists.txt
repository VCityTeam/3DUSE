if( NOT BUILD_CityGMLCutQtPlugin )
  return()
endif()

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
  set( CityGMLCut_Qt_Plugin_LIB
    ${CityGMLCut_Qt_Plugin_LIB}
    ${QT_LIBRARIES}
  )
endif()

add_library( CityGMLCutQtPlugin SHARED
  ${CityGMLCut_Qt_Plugin_HEADER}
  ${CityGMLCut_Qt_Plugin_SRC}
  ${EMBARKED_OSGQT_SRC}
  CityGMLCutQtPlugin.cxx
  Dialogs/DialogCityGMLCut1.cxx
  ${CMAKE_SOURCE_DIR}/src/utils/cmdline/cityGMLCut.cxx  #FIXME: why does a plugin depend on cmdline?
  ${CMAKE_SOURCE_DIR}/src/utils/cmdline/triangulate.cxx #FIXME: why does a plugin depend on cmdline?
)

if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set_target_properties( CityGMLCutQtPlugin PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}
      LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/Lib
    )
  endforeach()
endif()

target_include_directories( CityGMLCutQtPlugin PUBLIC
  ${CMAKE_SOURCE_DIR}/src/gui              # For moc on Generic_PluginInterface
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CITYGML_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

target_link_libraries( CityGMLCutQtPlugin
  ${CityGMLCut_Qt_Plugin_LIB}
  ${VCITY_GUI_LIB}
)
if( BUILD_GUI_QT5 )
  target_include_directories( CityGMLCutQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLCutQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

# FIXME: temporarily disabled because of AppVeyor build crash
# install( TARGETS CityGMLCutQtPlugin
#   LIBRARY DESTINATION . COMPONENT ${PRJ_NAME}
# )
