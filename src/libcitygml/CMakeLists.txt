#add_subdirectory(utils)

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
  parser.cpp
  parserlibxml2.cpp
  parserparams.cpp
  #FIXME: remove the following empty file and the xerces dependency as well ?
  # parserxercesc.cpp 
  polygon.cpp
  temporalExt.cpp
  tesselator.cpp
  texture.cpp

  export/exportCityGML.cpp
  export/exportJSON.cpp
  export/exportOBJ.cpp
  export/exporter.cpp

  import/importer.cpp
  import/importerAssimp.cpp

  # FIXME : ADE should be in separate libraries (possibly as libCityGML plugins)
  ADE/ADE.cpp
  ADE/temporal/temporalHandler.cpp
  ADE/temporal/transaction.cpp
  ADE/temporal/version.cpp
  ADE/temporal/versionTransition.cpp
)

target_include_directories( citygml PUBLIC ${CMAKE_SOURCE_DIR}/src )
# Required for generated citygml_export.h
target_include_directories( citygml PUBLIC ${CMAKE_CURRENT_BINARY_DIR} )

target_link_libraries( citygml
  ${ASSIMP_LIBRARY}
  ${LIBXML2_LIBRARIES}
  ${OPENGL_LIBRARIES}    # FIXME: tesselation dependency 
  ${OPENTHREADS_LIBRARY}
  ${OSG_LIBRARIES}       # FIXME: this dependency is a nonsense
  ${QT_LIBRARIES}        # FIXME: this dependency is a nonsense too
)

# Dealing with VC++ oddities: generates the export header citygml_exports.h 
include (GenerateExportHeader)
GENERATE_EXPORT_HEADER( citygml ) 
