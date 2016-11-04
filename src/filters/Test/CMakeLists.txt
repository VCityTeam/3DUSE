if( NOT BUILD_TESTING )
  return()
endif()

add_subdirectory( PCL )

###### Lonely tests:
add_executable( TestComputeEnvelope TestComputeEnvelope.cpp )

target_include_directories( TestComputeEnvelope PUBLIC
  ${CITYGMLUTILS_INCLUDE_DIR}
  ${FILTERS_INCLUDE_DIR}
)

target_link_libraries( TestComputeEnvelope
  ${CITYGMLUTIlS_LIBRARIES}
  ${FILTERS_LIBRARIES}
)

if( NOT WIN32 OR BUILD_UNMATURE_TESTS )
  # Fails on Appveyor at run time with "Command exited with code -1073741515"
  # error message (which seems to indicate that a DLL was not found). The 
  # problem is thus not the test but the Appveyor context and probably
  # due to an uncomplete PATH environment variable. 
  add_test( TEST_COMPUTE_ENVELOPE
    TestComputeEnvelope
    ${CMAKE_CURRENT_SOURCE_DIR}/Data/ComputeEnvelope_Lyon_1er_00136_Cut_Buildings_with_textures.gml
    ${CMAKE_CURRENT_SOURCE_DIR}/Data/ComputeEnvelope_Shape.shp
    ${CMAKE_CURRENT_BINARY_DIR}/Data/ComputeEnvelop_Output_Computed.gml
  )
endif()
