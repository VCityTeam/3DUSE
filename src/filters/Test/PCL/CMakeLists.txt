if( NOT BUILD_PCL )
  return()
endif()

#######
add_executable( TestPCLWritePcdFile TestPCLWritePcdFile.cpp )
target_include_directories( TestPCLWritePcdFile PUBLIC ${PCL_INCLUDE_DIRS})

if( WIN32 )
  set_target_properties( TestPCLWritePcdFile PROPERTIES
                         COMPILE_DEFINITIONS ${PCL_DEFINITIONS} )
  target_compile_definitions( TestPCLWritePcdFile
                              PRIVATE ${PCL_DEFINITIONS_VCITY} )
endif()

target_link_libraries( TestPCLWritePcdFile ${PCL_LIBRARIES} )
add_test( TEST_PCL_WRITE_PCD_FILE TestPCLWritePcdFile )

#######
add_executable( TestPCLExtractVegetionLidar TestPCLExtractVegetionLidar.cpp )
target_include_directories( TestPCLExtractVegetionLidar PUBLIC
  ${VCITY_INCLUDE_DIR}
  ${VCITYCORE_INCLUDE_DIR}
  ${CITYGML_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

if( WIN32 )
  set_target_properties( TestPCLExtractVegetionLidar PROPERTIES
                         COMPILE_DEFINITIONS ${PCL_DEFINITIONS} )
  target_compile_definitions( TestPCLExtractVegetionLidar
                              PRIVATE ${PCL_DEFINITIONS_VCITY} )
endif()

target_link_libraries( TestPCLExtractVegetionLidar
  ${GDAL_LIBRARY}
  ${LASLIB_LIBRARY}
  ${PCL_LIBRARIES} ${PCL_SEARCH_LIBRARY} ${PCL_SEGMENTATION_LIBRARY}
  ${CITYGMLUTILS_LIBRARIES}
)

message("aaaaaaaaaaaaa BUILD_UNMATURE_TESTS " ${BUILD_UNMATURE_TESTS})
message("bbbbbbbbbbbbb WIN32 :" ${WIN32})
message("ccccccccccccc LINUX :" ${LINUX})
message("ccccccccccccc APPLE :" ${APPLE})
message("ddddddddddddd CMAKE_CXX_COMPILER_ID " ${CMAKE_CXX_COMPILER_ID} )

if( CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
message("eeeeeeeeeeeee CMAKE_CXX_COMPILER_ID MATCHES Clang: YES " )
else()
message("eeeeeeeeeeeee CMAKE_CXX_COMPILER_ID MATCHES Clang: NO " )
endif()

if( CMAKE_BUILD_TYPE MATCHES DEBUG )
message("fffffffffffff CMAKE_BUILD_TYPE MATCHES DEBUG : YES " )
else()
message("fffffffffffff CMAKE_BUILD_TYPE MATCHES DEBUG : NO " )
endif()

if( LINUX AND ( CMAKE_CXX_COMPILER_ID MATCHES "Clang") 
          AND ( CMAKE_BUILD_TYPE MATCHES DEBUG ) ) 
message("gggggggggggg THE LINUX CONTEXT : YES " )
else()
message("gggggggggggg THE LINUX CONTEXT : NO " )
endif()

if(   BUILD_UNMATURE_TESTS
    OR
      NOT (
        WIN32 
        # Fails on Appveyor at run time with "Command exited with code -1073741515"
        # error message (which seems to indicate that a DLL was not found). The 
        # problem is thus not the test but the Appveyor context and probably
        # due to an uncomplete PATH environment variable. 
      OR
        ( LINUX AND ( CMAKE_CXX_COMPILER_ID MATCHES "Clang") 
                AND ( CMAKE_BUILD_TYPE MATCHES DEBUG ) )
        # Segfaults on Travis (Ubuntu ???) with clan when on Debug mode (after 26s
        # without any debug message). Note that it runs smoothly in release mode.
        # Unable to reproduce any segfault in interactive mode on an
        # Ubuntu 14.04.3 LTS server.
      )
  )
message("hhhhhhhhhhhhhhhhh BUILDING TEST " )
  add_test( TEST_PCL_EXTRACT_VEGETION_LIDAR  TestPCLExtractVegetionLidar
    ${CMAKE_CURRENT_SOURCE_DIR}/Data/IRCFilteredLidarForExtractingVegetation.las
    ${CMAKE_CURRENT_BINARY_DIR}/IRCFilteredLidarForExtractingVegetationOutput.gml
  )
endif()
