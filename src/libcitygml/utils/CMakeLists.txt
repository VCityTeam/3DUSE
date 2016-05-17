
add_library( citygmlutils SHARED
  # FIXME: compiled with a GLOB within the project CMakeLists.txt
  # FIXME AABB.cpp
  # FIXME Triangle.cpp
  ../export/exportJSON.cpp
)

target_include_directories( citygmlutils PUBLIC
  ${CITYGML_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries( citygmlutils
  citygml
  # FIXME ${OPENGL_glu_LIBRARY}
)
