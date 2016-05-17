OPTION( BUILD_CityGMLCutQtPlugin "BUILD CityGMLCutQtPlugin " ON)

if (BUILD_CityGMLCutQtPlugin)
	set(CityGMLCut_Qt_Plugin_HEADER
    CityGMLCutQtPlugin.h
    Dialogs/DialogCityGMLCut1.h
  )
	set(CityGMLCut_Qt_Plugin_UI Dialogs/DialogCityGMLCut1.ui )

	if (WITH_QT5)
	  ADD_DEFINITIONS("-DWITH_QT5")
	  QT5_WRAP_CPP( CityGMLCut_Qt_Plugin_MOC_CPP ${CityGMLCut_Qt_Plugin_HEADER} )
	  QT5_WRAP_UI ( CityGMLCut_Qt_Plugin_UI_CPP  ${CityGMLCut_Qt_Plugin_UI} )
	  set(CityGMLCut_Qt_Plugin_SRC
       ${CityGMLCut_Qt_Plugin_SRC}
       ${CityGMLCut_Qt_Plugin_MOC_CPP}
       ${CityGMLCut_Qt_Plugin_UI_CPP})

	else(WITH_QT5) 
	  QT4_WRAP_CPP( CityGMLCut_Qt_Plugin_MOC_CPP ${CityGMLCut_Qt_Plugin_HEADER})
	  QT4_WRAP_UI ( CityGMLCut_Qt_Plugin_UI_CPP  ${CityGMLCut_Qt_Plugin_UI})
	  set(CityGMLCut_Qt_Plugin_SRC
      ${CityGMLCut_Qt_Plugin_SRC}
      ${CityGMLCut_Qt_Plugin_MOC_CPP}
      ${CityGMLCut_Qt_Plugin_UI_CPP})
	  set(CityGMLCut_Qt_Plugin_LIB ${CityGMLCut_Qt_Plugin_LIB} ${QT_LIBRARIES})
	endif(WITH_QT5)

  add_library( CityGMLCutQtPlugin SHARED
    CityGMLCutQtPlugin.cxx
    Dialogs/DialogCityGMLCut1.cxx
    ${CMAKE_SOURCE_DIR}/src/utils/cmdline/cityGMLCut.cxx # FIXME
    ${CMAKE_SOURCE_DIR}/src/utils/cmdline/triangulate.cxx # FIXME
    ${CityGMLCut_Qt_Plugin_HEADER}
    ${CityGMLCut_Qt_Plugin_SRC}
    ${VCITY_GUI_SRC_WIN}
  )

	target_include_directories( CityGMLCutQtPlugin PUBLIC
    ${CITYGML_INCLUDE_DIR}
  )

	target_link_libraries( CityGMLCutQtPlugin
    ${CityGMLCut_Qt_Plugin_LIB}
    ${VCITY_GUI_LIB}
  )

	if (WITH_QT5)
	  QT5_USE_MODULES(CityGMLCutQtPlugin Core Widgets OpenGL)
	endif(WITH_QT5)

endif (BUILD_CityGMLCutQtPlugin)
