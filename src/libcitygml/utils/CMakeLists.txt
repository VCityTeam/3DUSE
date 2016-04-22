
add_library( citygmlutils
  AABB.cpp
  Triangle.cpp
)
target_include_directories( citygmlutils PUBLIC ${CITYGML_INCLUDE_DIR} )
target_include_directories( citygmlutils PUBLIC ${CMAKE_SOURCE_DIR}/src )
