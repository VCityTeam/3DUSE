if( NOT BUILD_PCL )
  return()
endif()

add_executable( TestPCLWritePcdFile TestPCLWritePcdFile.cpp )

target_include_directories( TestPCLWritePcdFile PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries( TestPCLWritePcdFile ${PCL_LIBRARIES} )

add_test( TEST_PCL_WRITE_PCD_FILE TestPCLWritePcdFile )
