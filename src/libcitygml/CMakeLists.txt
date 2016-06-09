add_subdirectory(utils)

if(APPLE)
  # FIXME: instead of hiding the deprecation messages update the code ?
  # In order to avoid 'glu*' related deprecation messages...
  set_source_files_properties( tesselator.cpp
    PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations"
  ) 
endif(APPLE)

add_library( citygml SHARED
  # FIXME: the following are dependencies of import/importerASC.cpp
  # FIXME ${CMAKE_SOURCE_DIR}/src/DataStructures/DEM/osgMnt.cpp   # WTF
  # FIXME ${CMAKE_SOURCE_DIR}/src/utils/OGRGDAL_Utils/OGRGDALtools.cpp 
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
  #FIXME: the following file dates back to the original version of CityGML that
  # suppported both libxml2 and xerces. This fork evolved the libxml2 with the
  # additions of ADE but the xerces version was not maintained/evolved. 
  # Discuss whether we should definitively give up on xerces support.
  # Note: xerces as the reputation of having a smaller memory footprint than
  # libxml2 that imports the whole content in memory before parsing per se. This
  # libxml2 way of doing thins might be a strong constrain when working with huge
  # citygml files.
  # If xerces support is dropped then remove the following file, and allo remove
  # the xerces dependency at the cmakefile level (Find_package( xerces )...)
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

  ADE/document/documentHandler.cpp
  ADE/document/documentObject.cpp
  ADE/document/reference.cpp
  ADE/document/tag.cpp
)

# The dependence of this library towards QT both undue and a drag:
if (WITH_QT5)
  QT5_USE_MODULES( citygml Core Widgets Xml OpenGL )
  # FIXME: assert the following two lines are indeed required
  add_definitions("-DWITH_QT5")
  # FIXME: if this line were to be kept remove the semi-colon from last argument
  set_target_properties( citygml PROPERTIES COMPILE_DEFINITIONS ";WITH_QT5" )
endif ()

# FIXME: this instruction should and can probably be compeletely removed !
target_include_directories( citygml PUBLIC 
 ${CMAKE_SOURCE_DIR}
 ${CMAKE_SOURCE_DIR}/src
 ${CMAKE_SOURCE_DIR}/src/processes     # FIXME: to be removed with ToolAlgoCut.cpp
 ${CMAKE_SOURCE_DIR}/src/DataStructures/DEM/  # FIXME: remove with osgMnt.cpp
)

target_include_directories( citygml PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}       # Required for generated citygml_export.h
  ${VCITYCORE_INCLUDE_DIR}
)

target_link_libraries( citygml
  ${ASSIMP_LIBRARY}
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
  ${VCITYCORE_LIBRARY}
)

# Dealing with VC++ oddities: generates the export header citygml_exports.h 
include (GenerateExportHeader)
GENERATE_EXPORT_HEADER( citygml ) 
