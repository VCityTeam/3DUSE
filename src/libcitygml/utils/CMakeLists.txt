
add_library( citygmlutils SHARED
  # FIXME: compiled at the project CMakeLists.txt level. Refer there for comment.
  # FIXME AABB.cpp
  # FIXME Triangle.cpp
  CityGMLtools.cpp
  exportJSON.cpp
  exportOBJ.cpp
  importerASC.cpp
  importerAssimp.cpp
  TiledFilesLayout.cpp
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
