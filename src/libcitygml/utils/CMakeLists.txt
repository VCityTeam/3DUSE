
add_library( citygmlutils SHARED
  # FIXME: compiled with a GLOB within the project CMakeLists.txt
  # FIXME AABB.cpp
  # FIXME Triangle.cpp
  exportJSONTest.cpp
)

target_include_directories( citygmlutils PUBLIC
  ${CITYGML_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR} # for generated citygmlutils_export.h
  ${CMAKE_SOURCE_DIR}/src     # FIXME: assert whether this is needed
)

target_link_libraries( citygmlutils
  citygml
  # FIXME ${OPENGL_glu_LIBRARY}
)

# Dealing with VC++ oddities: generates the export header citygmlutils_exports.h 
include (GenerateExportHeader)
GENERATE_EXPORT_HEADER( citygmlutils )
