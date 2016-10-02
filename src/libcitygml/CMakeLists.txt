add_subdirectory( utils )
add_subdirectory( Test  )

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
  polygon.cpp
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

if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set_target_properties( citygml PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}
      LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/Lib
    )
  endforeach()
endif()

target_include_directories( citygml PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}       # Required for generated citygml_export.h
)

target_link_libraries( citygml
  ${ASSIMP_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  ${GDAL_LIBRARY}
  ${OPENGL_LIBRARIES}    # FIXME: tesselation dependency 
  ${OSG_LIBRARIES}       # FIXME: this is a weak dependency on
                         # - osg::Referenced::signalObserversAndDelete
)

# FIXME The dependence of this library towards QT is both undue and a drag.
# This dependence is due to two files:
#  - exporter.cpp (uses a QTdate)
if( BUILD_GUI_QT4 )
  target_link_libraries( citygml ${QT_LIBRARIES} )
endif()

if( BUILD_GUI_QT5 )
  target_include_directories( citygml PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>
  )
  target_link_libraries( citygml Qt5::Core )
endif ()

# Dealing with VC++ oddities: generates the export header citygml_exports.h 
GENERATE_EXPORT_HEADER( citygml ) 
