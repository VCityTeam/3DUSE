# add_subdirectory(core)
add_subdirectory( libfilters )
add_subdirectory( libcitygml )

add_library( vcitycore SHARED
  DataStructures/DEM/osgMnt.cpp 
  utils/OGRGDAL_Utils/OGRGDALtools.cpp
  utils/OGRGDAL_Utils/OGRGDALtoShpWriter.cpp
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
  ${QT_LIBRARIES}   # FIXME: required by SaveGeometrytoShape from
                    # OGRGDALtoShpWriter.cpp on QDIR and QString.
)

target_include_directories( vcitycore PUBLIC
  ${CITYGML_INCLUDE_DIR}         # FIXME: OGRGDALtools.hpp includes vecs.hpp !
  ${CMAKE_CURRENT_BINARY_DIR}    # For generated vcitycore_export.h
)

# Dealing with VC++ oddities: generates the export header core_exports.h 
include (GenerateExportHeader)
GENERATE_EXPORT_HEADER( vcitycore )
