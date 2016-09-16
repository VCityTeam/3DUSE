if( NOT BUILD_CityGMLVisibiliteQtPlugin )
  return()
endif()

set( CityGMLVisibilite_Qt_Plugin_SRC
  data/BelvedereDB.cxx
  data/ViewPoint.cxx

  AlignementTree.cxx
  CityGMLVisibiliteQtPlugin.cxx
  Export.cxx
  FlatRoof.cxx
  MonoTileAnalysis.cxx
  MultiTileAnalysis.cxx
  VegetTool.cxx
)

set( CityGMLVisibilite_Qt_Plugin_HEADER
  CityGMLVisibiliteQtPlugin.h
  Dialogs/dialogVisibilite.hpp
)

set( CityGMLVisibilite_Qt_Plugin_UI
  Dialogs/dialogVisibilite.ui
)

if( BUILD_GUI_QT5 )
  ADD_DEFINITIONS("-DBUILD_GUI_QT5")
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
endif()

if( BUILD_GUI_QT4 )
  QT4_WRAP_CPP( CityGMLVisibilite_Qt_Plugin_MOC_CPP
    ${CityGMLVisibilite_Qt_Plugin_HEADER}
    TARGET CityGMLVisibiliteQtPlugin
  )
  QT4_WRAP_UI( CityGMLVisibilite_Qt_Plugin_UI_CPP
    ${CityGMLVisibilite_Qt_Plugin_UI}
  )
  set(CityGMLVisibilite_Qt_Plugin_SRC
    ${CityGMLVisibilite_Qt_Plugin_SRC}
    ${CityGMLVisibilite_Qt_Plugin_MOC_CPP}
    ${CityGMLVisibilite_Qt_Plugin_UI_CPP}
  )
endif()

add_library( CityGMLVisibiliteQtPlugin SHARED
   CityGMLVisibiliteQtPlugin.cxx
   Dialogs/dialogVisibilite.cxx
   ${CityGMLVisibilite_Qt_Plugin_HEADER}
   ${CityGMLVisibilite_Qt_Plugin_SRC}
)

if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set_target_properties( CityGMLVisibiliteQtPlugin PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}
      LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/Lib
    )
  endforeach()
endif()

target_include_directories( CityGMLVisibiliteQtPlugin PUBLIC
  src                            # For moc on Generic_PluginInterface
  src/libfilters
  ${CMAKE_SOURCE_DIR}            # FIXME: because VegetTool.hpp includes
                                 # src/core/application.hpp
  ${CITYGML_INCLUDE_DIR}
  ${FILTERS_INCLUDE_DIR} 
)
target_link_libraries( CityGMLVisibiliteQtPlugin
  VCityGui
  ${LINUX_FLAGS}
  ${CITYGML_LIBRARIES}
  ${CITYGMLUTILS_LIBRARIES}
  ${FILTERS_LIBRARIES}
  ${QT_LIBRARIES}           # FIXME: looks suspicious since we have a 
                            # special treatment for QT just below
  ${VCITY_GUI_LIB}
)

if( BUILD_GUI_QT5 )
  target_include_directories( CityGMLVisibiliteQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLVisibiliteQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

if( NOT ( BUILD_EMBARKED_OSG-QT_32 OR BUILD_EMBARKED_OSG-QT_34 ) )
  target_link_libraries( CityGMLVisibiliteQtPlugin ${OSGQT_LIBRARIES} )
endif()

# FIXME: temporarily disabled because of AppVeyor build crash
# install( TARGETS CityGMLVisibiliteQtPlugin
#   LIBRARY DESTINATION . COMPONENT ${PRJ_NAME}  # FIXME: is this line needed ?
# )
