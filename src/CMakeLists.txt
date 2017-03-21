add_subdirectory( Examples )
add_subdirectory( filters )
add_subdirectory( plugins )
add_subdirectory( libcitygml )
add_subdirectory( utils )

# FIXME: to be finished...
# The following vcitycore library might have a missleading name that
# might trick into thinking that the library holds the (build) content
# of the src/core directory. Yet the src/core directory is not really
# the core of a possible CLI/GUI independent core code but really the
# "core" of the GUI application (i.e. files that do not include QT files).
# Besides the content of src/core gets build by the main CMakeLists.txt
# and not as a separated library (this last point is logic since src/core
# and src/gui must be build together: that is, it wouldn't make sense to
# link src/core against something else that src/gui...)
# In order to finish the job one should really:
#  - cd src
#  - mkdir viewerApp
#  - mv gui core guiApp
#  - mkdir core
#  - mv utils DataStructures core 
# Then:
#  * deport the content concerning viewerApp out of the main CMakeLists.txt
#    to a newly created viewerApp/CMakeLists.txt.
#  * basically move the content of the present makefile to core/CMakeLists.txt
# add_subdirectory(core)   

add_library( vcitycore SHARED
  DataStructures/DEM/osgMnt.cpp 
  DataStructures/quaternion.cpp
  utils/OGRGDAL_Utils/OGRGDALtools.cpp
  utils/OGRGDAL_Utils/OGRGDALtoShpWriter.cpp
)

if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set_target_properties( vcitycore PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}
      LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/Lib
    )
  endforeach()
endif()

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

# FIXME: within OGRGDALtoShpWriter.cpp the SaveGeometrytoShape() funcion
# uses QT's QDIR and QString which creates a dependency of the backend
# towards the frontend. Switch to boost::filesystem and get rid of QT.
if( BUILD_GUI_QT5 )
  target_include_directories( vcitycore PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( vcitycore Qt5::Core )
endif()
if( BUILD_GUI_QT4 )
  target_link_libraries( vcitycore ${QT_LIBRARIES} )
endif()

# Dealing with VC++ oddities: generates the export header core_exports.h 
include (GenerateExportHeader)
GENERATE_EXPORT_HEADER( vcitycore )
