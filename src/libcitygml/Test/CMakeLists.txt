if( NOT BUILD_TESTING )
  return()
endif()

add_executable(        test_load_citygml_file test_load_file.cpp )
target_link_libraries( test_load_citygml_file ${CITYGML_LIBRARIES} )
add_test( CITYGML_LOAD_FILE_1
  test_load_citygml_file
  ${CMAKE_CURRENT_SOURCE_DIR}/Data/exemple_versions.gml
)
