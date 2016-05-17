add_subdirectory(utils)

if(APPLE)
  # FIXME: instead of hiding the deprecation messages update the code ?
  # In order to avoid 'glu*' related deprecation messages...
  set_source_files_properties( tesselator.cpp
    PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations"
  ) 
endif(APPLE)

add_library( citygml SHARED
  URI.cpp
  appearance.cpp
  appearancemanager.cpp
  citymodel.cpp
  cityobject.cpp
  envelope.cpp
  geometry.cpp
  georeferencedtexture.cpp
  linearring.cpp
  material.cpp
  object.cpp
  parserparams.cpp
  # FIXME: the following parserxercesc.cpp is obviously unused since we
  # can build the project without compiling it. It came from the original
  # IGN implementation but became deprecated within 3DUse derived work.
  # Remove this file (there doesn't seem to be any find_package concerning
  # xerces within any CMakeLists.txt)
  # parserxercesc.cpp 
  polygon.cpp
  temporalExt.cpp
  tesselator.cpp
  texture.cpp

  # XML binders (IO native to this library)
  parser.cpp
  parserlibxml2.cpp
  export/exportCityGML.cpp

  # Only used as base classes:
  export/exporter.cpp
  import/importer.cpp

  # FIXME : ADE should be in separate libraries (possibly as libCityGML plugins)
  # Note that the ADE is QT dependent on QString, QDateTime, qFree()...
  ADE/ADE.cpp
  ADE/temporal/temporalHandler.cpp
  ADE/temporal/transaction.cpp
  ADE/temporal/version.cpp
  ADE/temporal/versionTransition.cpp
)

# The dependence of this library towards QT both undue and a drag:
if (WITH_QT5)
  QT5_USE_MODULES( citygml Core Widgets Xml OpenGL )
  # FIXME: assert the following two lines are indeed required
  add_definitions("-DWITH_QT5")
  set_target_properties( citygml PROPERTIES COMPILE_DEFINITIONS ";WITH_QT5" )
endif ()

# Required for generated citygml_export.h
target_include_directories( citygml PUBLIC ${CMAKE_CURRENT_BINARY_DIR} )

target_link_libraries( citygml
  ${LIBXML2_LIBRARIES}
  ${GDAL_LIBRARY}
  ${OPENGL_LIBRARIES}    # FIXME: tesselation dependency 
  ${OPENTHREADS_LIBRARY} # FIXME: temporalExt.cpp dependency
  ${OSG_LIBRARIES}       # FIXME: this is a weak dependency on
                         # - osg::Referenced::signalObserversAndDelete
                         # - setOsg(osg::ref_ptr<osg::Group>) in temporalExt.cpp
  ${QT_LIBRARIES}        # FIXME: alas required by
                         # - exporter.cpp 
                         # - temporalExt.cpp 
)

# Dealing with VC++ oddities: generates the export header citygml_exports.h 
include (GenerateExportHeader)
GENERATE_EXPORT_HEADER( citygml ) 
