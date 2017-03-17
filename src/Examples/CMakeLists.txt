if( WITH_PCL_WITH_VISUALIZER )
  ## Optional example for pcl based visualisation (only available when PCL was
  ## build with VTK support)
  add_executable( ExamplePCLExtractBuildingFromLidar
    ExamplePCLExtractBuildingFromLidar.cpp
  )
  target_include_directories( ExamplePCLExtractBuildingFromLidar PUBLIC
    ${VCITY_INCLUDE_DIR}
    ${VCITYCORE_INCLUDE_DIR}
    ${CITYGML_INCLUDE_DIR}
    ${CITYGMLUTILS_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src/filters/Test/PCL  ## For PCLExtractBuildingFromLidar.tpp
  )
  
  if( WIN32 )
    set_target_properties( ExamplePCLExtractBuildingFromLidar PROPERTIES
                           COMPILE_DEFINITIONS ${PCL_DEFINITIONS} )
    target_compile_definitions( ExamplePCLExtractBuildingFromLidar
                                PRIVATE ${PCL_DEFINITIONS_VCITY} )
  endif()

message( "DEBUG-PCL Boost_LIBRARIES as seen from src/Examples/CMakeLists.txt"  ${Boost_LIBRARIES} )
  target_link_libraries( ExamplePCLExtractBuildingFromLidar
    ${GDAL_LIBRARY}
    ${LASLIB_LIBRARY}
    ${PCL_LIBRARIES}
      ${PCL_FILTERS_LIBRARY}
      ${PCL_FEATURES_LIBRARY}
      ${PCL_SAMPLE_CONSENSUS_LIBRARY}
      ${PCL_SEARCH_LIBRARY}
      ${PCL_SEGMENTATION_LIBRARY}
      ${PCL_SURFACE_LIBRARY}
      ${PCL_VISUALIZATION_LIBRARY}
    ${CITYGMLUTILS_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
endif()
