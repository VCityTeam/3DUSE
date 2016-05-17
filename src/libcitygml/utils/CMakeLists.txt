
add_library( citygmlutils SHARED
  AABB.cpp
  Triangle.cpp
)
target_include_directories( citygmlutils PUBLIC ${CITYGML_INCLUDE_DIR} )
target_include_directories( citygmlutils PUBLIC ${CMAKE_SOURCE_DIR}/src )

target_link_libraries( citygmlutils
  citygml
  ${OPENGL_glu_LIBRARY}
)
