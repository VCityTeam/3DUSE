if( NOT BUILD_PCL )
  return()
endif()

#######
add_executable( TestPCLWritePcdFile TestPCLWritePcdFile.cpp )
target_include_directories( TestPCLWritePcdFile PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries( TestPCLWritePcdFile ${PCL_LIBRARIES} )

if( WIN32 )
  # FIXME: is this really usefull ?
  set_target_properties( TestPCLWritePcdFile PROPERTIES
                         COMPILE_DEFINITIONS ${PCL_DEFINITIONS} )
  # To avoid pcl_macros.h(176): error C2065 M_LOG2E undeclared identifier
  set_target_properties( TestPCLWritePcdFile PROPERTIES
                         COMPILE_DEFINITIONS "_USE_MATH_DEFINES" )
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
