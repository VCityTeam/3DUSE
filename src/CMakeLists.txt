add_subdirectory( filters )
add_subdirectory( plugins )
add_subdirectory( libcitygml )
# add_subdirectory(core)   # FIXME: finish or clean up.

add_library( vcitycore SHARED
  DataStructures/DEM/osgMnt.cpp 
  utils/OGRGDAL_Utils/OGRGDALtools.cpp
  # FIXME: within OGRGDALtoShpWriter.cpp the SaveGeometrytoShape() funcion
  # uses QT's QDIR and QString which creates a dependency of the backend
  # towards the frontend. Switch to boost::filesystem and get rid of QT.
  utils/OGRGDAL_Utils/OGRGDALtoShpWriter.cpp
)

target_include_directories( vcitycore PUBLIC
  ${CITYGML_INCLUDE_DIR}         # FIXME: OGRGDALtools.hpp includes vecs.hpp !
  ${CMAKE_CURRENT_BINARY_DIR}    # For generated vcitycore_export.h
)

target_link_libraries( vcitycore
  ${OSG_LIBRARIES}
  ${OPENTHREADS_LIBRARIES}      # FIXME osgMnt.cpp depends on osg::BufferData
                 # which in turn depends on OpenThreads::Atomic::operator++
                 # and operator--. If this is an undirect dependency of
                 # of the OSG_LIBRARIES then change this project's definition
                 # of this variable by extending it systematically with
                 # ${OPENTHREADS_LIBRARIES} and remove this line (and
                 # other of its occurences). 
  ${GDAL_LIBRARY}
)


if( WITH_QT5 )
  # FIXME: following line produces cmake warnings and was deprecated: refer
  # to https://bugreports.qt.io/browse/QTBUG-39457
  # QT5_USE_MODULES( vcitycore Core )  # QT4 way of providing QDir/QString 
  # The replacement is a bit verbose, but no other workaround according to
  # https://cmake.org/Bug/view.php?id=14778
  target_include_directories( vcitycore PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( vcitycore Qt5::Core )
else() # With QT4
  target_link_libraries( vcitycore ${QT_LIBRARIES} )
endif()

# Dealing with VC++ oddities: generates the export header core_exports.h 
include (GenerateExportHeader)
GENERATE_EXPORT_HEADER( vcitycore )
