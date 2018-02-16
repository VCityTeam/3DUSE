configure_file( 
  Application-config.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}-config.cpp
  @ONLY )

add_executable( ${APP_NAME}-config ${APP_NAME}-config.cpp )
add_test( TEST_PRINT_CONFIG ${APP_NAME}-config )

### Extract Buildings Construction Demolition Dates
add_executable( extractBuildingDates
  extractBuildingsConstructionDemolitionDates.cxx
)

if( WIN32 )
  target_compile_definitions( extractBuildingDates PUBLIC
                              "-DBOOST_ALL_DYN_LINK")
endif()

target_link_libraries(extractBuildingDates
  vcitycore     ## For SaveGeometrytoShape()
  citygmlutils  ## For the Tile class
  filters       ## For ChangeDetection()
  ${Boost_LIBRARIES}
)
