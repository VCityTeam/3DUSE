add_executable( TestComputeEnvelope TestComputeEnvelope.cpp )

target_include_directories( TestComputeEnvelope PUBLIC
  ${CITYGMLUTILS_INCLUDE_DIR}
  ${FILTERS_INCLUDE_DIR}
)

target_link_libraries( TestComputeEnvelope
  ${CITYGMLUTIlS_LIBRARIES}
  ${FILTERS_LIBRARIES}
)

add_test( TEST_COMPUTE_ENVELOPE
  TestComputeEnvelope
  ${CMAKE_CURRENT_SOURCE_DIR}/Data/ComputeEnvelope_Lyon_1er_00136_Cut_Buildings_with_textures.gml
  ${CMAKE_CURRENT_SOURCE_DIR}/Data/ComputeEnvelope_Shape.shp
  ${CMAKE_CURRENT_BINARY_DIR}/Data/ComputeEnvelop_Output_Computed.gml
)
