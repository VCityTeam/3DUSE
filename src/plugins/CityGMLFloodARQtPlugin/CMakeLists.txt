if( NOT BUILD_CityGMLFloodARQtPlugin ) 
  return()
endif()

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
  set(CityGMLFloodAR_Qt_Plugin_LIB ${CityGMLFloodAR_Qt_Plugin_LIB} ${QT_LIBRARIES})
endif()

add_library( CityGMLFloodARQtPlugin SHARED
  ${CityGMLFloodAR_Qt_Plugin_HEADER}
  ${CityGMLFloodAR_Qt_Plugin_SRC}
  ${EMBARKED_OSGQT_SRC}
  CityGMLFloodARQtPlugin.cxx
  Dialogs/dialogFloodAR.cxx
  FloodARTools.cxx
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

if( BUILD_GUI_QT5 )
  target_include_directories( CityGMLFloodARQtPlugin PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( CityGMLFloodARQtPlugin Qt5::Widgets Qt5::OpenGL )
endif()

if( NOT ( BUILD_EMBARKED_OSG-QT_32 OR BUILD_EMBARKED_OSG-QT_34 ) )
  target_link_libraries( CityGMLFloodARQtPlugin ${OSGQT_LIBRARIES} )
endif()

# FIXME: temporarily disabled because of AppVeyor build crash
# install( TARGETS CityGMLFloodARQtPlugin
#   LIBRARY DESTINATION lib   # FIXME: inquire on this
#     COMPONENT ${PRJ_NAME}     # FIXME: is this line needed ?
#)
