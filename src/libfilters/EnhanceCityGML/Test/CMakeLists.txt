add_executable(TestLinkCityGMLShape TestLinkCityGMLShape.cpp ${CMAKE_SOURCE_DIR}/src/libfilters/EnhanceCityGML/LinkCityGMLShape.cpp)
target_link_libraries(TestLinkCityGMLShape
  ${VCITYCORE_LIBRARY}
  ${CITYGML_LIBRARIES}
  ${CITYGMLUTILS_LIBRARIES})
		
add_test( LINKCITYGMLSHAPE
		  TestLinkCityGMLShape
		)
		
target_include_directories( TestLinkCityGMLShape PUBLIC
  ${CITYGML_INCLUDE_DIR}
  ${FILTERS_INCLUDE_DIR} 
  ${CMAKE_SOURCE_DIR}/src
  ${CITYGMLUTILS_INCLUDE_DIR}
  ${VCITYCORE_INCLUDE_DIR}
)