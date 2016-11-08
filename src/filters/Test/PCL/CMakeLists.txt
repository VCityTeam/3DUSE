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

#########################################################
# CLEANME UP FIXME
message("aaaaaaaaaaaaa BUILD_UNMATURE_TESTS " ${BUILD_UNMATURE_TESTS})
message("bbbbbbbbbbbbb WIN32 :" ${WIN32})
message("ccccccccccccc UNIX  :" ${UNIX})
message("ccccccccccccc APPLE :" ${APPLE})
message("ddddddddddddd CMAKE_CXX_COMPILER_ID " ${CMAKE_CXX_COMPILER_ID} )

if( CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
  message("eeeeeeeeeeeee CMAKE_CXX_COMPILER_ID MATCHES Clang: YES " )
else()
  message("eeeeeeeeeeeee CMAKE_CXX_COMPILER_ID MATCHES Clang: NO " )
endif()

if( CMAKE_BUILD_TYPE MATCHES "Debug" )
  message("fffffffffffff CMAKE_BUILD_TYPE MATCHES DEBUG : YES " )
else()
  message("fffffffffffff CMAKE_BUILD_TYPE MATCHES DEBUG : NO " )
endif()

if(     WIN32 AND ( CMAKE_BUILD_TYPE      MATCHES "Debug" ) )
  message("gggggggggggg THE WIN32 CONTEXT : YES " )
else()
  message("gggggggggggg THE WIN32 CONTEXT : NO " )
endif()

if( UNIX AND (NOT APPLE)
         AND ( CMAKE_CXX_COMPILER_ID MATCHES "Clang") 
         AND ( CMAKE_BUILD_TYPE      MATCHES "Debug" ) ) 
  message("hhhhhhhhhhhh THE UNIX CONTEXT : YES " )
else()
  message("hhhhhhhhhhhh THE UNIX CONTEXT : NO " )
endif()
#########################################################

if(   BUILD_UNMATURE_TESTS
    OR
      NOT (
        ( WIN32 AND ( CMAKE_BUILD_TYPE      MATCHES "Debug" ) )
        # Hangs out...
      OR
        ( UNIX AND ( NOT APPLE ) 
               AND ( CMAKE_CXX_COMPILER_ID MATCHES "Clang" ) 
               AND ( CMAKE_BUILD_TYPE      MATCHES "Debug" )
        )
        # Segfaults on Travis (Ubuntu ???) with clan when on Debug mode (after 26s
        # without any debug message). Note that it runs smoothly in release mode.
        # Unable to reproduce any segfault in interactive mode on an
        # Ubuntu 14.04.3 LTS server.
      )
  )
  add_test( TEST_PCL_EXTRACT_VEGETION_LIDAR  TestPCLExtractVegetionLidar
    ${CMAKE_CURRENT_SOURCE_DIR}/Data/IRCFilteredLidarForExtractingVegetation.las
    ${CMAKE_CURRENT_BINARY_DIR}/IRCFilteredLidarForExtractingVegetationOutput.gml
  )
else()
  message("  Test TestPCLExtractVegetionLidar shall NOT be run." )
endif()
