
add_library( citygmlutils SHARED
  AABB.cpp
  CityGMLtools.cpp
  exportJSON.cpp
  exportOBJ.cpp
  importerASC.cpp
  importerAssimp.cpp
  TiledFilesLayout.cpp

  # FIXME: Triangle.cpp depends on core/tile.hpp, that in turn depends
  # on citygml. Hence we can import core/tile.cpp without adding further
  # dependency to this citygmlutils lib.
  # Yet one must move tile.cpp to this directory
  ../../core/tile.cpp
  Triangle.cpp
)

target_include_directories( citygmlutils PUBLIC
  ${CITYGML_INCLUDE_DIR}
  ${VCITYCORE_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR} # for generated citygmlutils_export.h
)

target_link_libraries( citygmlutils
   ${CITYGML_LIBRARIES}
   ${VCITYCORE_LIBRARY}
)

# Dealing with VC++ oddities: generates the export header citygmlutils_exports.h 
include (GenerateExportHeader)
GENERATE_EXPORT_HEADER( citygmlutils )
