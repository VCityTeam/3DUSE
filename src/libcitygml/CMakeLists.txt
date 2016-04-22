add_subdirectory(utils)

include_directories( ${CMAKE_SOURCE_DIR}/src )

if (APPLE)
  # FIXME: instead of hiding the deprecation messages update the code ?
  # In order to avoid 'glu*' related deprecation messages...
  set_source_files_properties( tesselator.cpp
    PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations"
  ) 
endif (APPLE)

add_library( citygml # SHARED
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
  #FIXME: remove the following file and the xerces dependency 
  # as well ?
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

  # FIXME Refer to above comment on ADE
  ADE/ADE.cpp
  ADE/temporal/temporalHandler.cpp
  ADE/temporal/transaction.cpp
  ADE/temporal/version.cpp
  ADE/temporal/versionTransition.cpp
)

target_link_libraries( citygml ${LIBXML2_LIBRARIES} )
