if( NOT BUILD_PCL )
  return()
endif()

#######
add_executable( TestPCLWritePcdFile TestPCLWritePcdFile.cpp )
target_include_directories( TestPCLWritePcdFile PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries( TestPCLWritePcdFile ${PCL_LIBRARIES} )

if( WIN32 )
  set_target_properties( TestPCLWritePcdFile PROPERTIES
                         COMPILE_DEFINITIONS ${PCL_DEFINITIONS} )
  target_compile_definitions( TestPCLWritePcdFile
                              PRIVATE ${PCL_DEFINITIONS_VCITY} )
endif()

add_test( TEST_PCL_WRITE_PCD_FILE TestPCLWritePcdFile )

#######
if( FALSE )
add_executable( TestPCLExtractVegetionLidar TestPCLExtractVegetionLidar.cpp )
if( WIN32 )
set_target_properties( TestPCLExtractVegetionLidar PROPERTIES
                       COMPILE_DEFINITIONS ${PCL_DEFINITIONS} )
endif()
target_include_directories( TestPCLExtractVegetionLidar PUBLIC
                            ${PCL_INCLUDE_DIRS})
target_link_libraries( TestPCLExtractVegetionLidar ${PCL_LIBRARIES} )
add_test(  TEST_PCL_EXTRACT_VEGETION_LIDAR  TestPCLExtractVegetionLidar )
endif()
