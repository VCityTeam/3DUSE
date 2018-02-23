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
  # Fix the odd interference between find_package( Boost) and the
  # VC++ auto-linking mechanisms that is documented e.g. here:
  # http://stackoverflow.com/questions/6468681/problem-linking-to-boost-library-with-cmake-generated-project-file-for-msvc9
  target_compile_definitions( extractBuildingDates PRIVATE
                              "BOOST_ALL_DYN_LINK=1" )
endif()

target_link_libraries(extractBuildingDates
  vcitycore     ## For SaveGeometrytoShape()
  citygmlutils  ## For the Tile class
  filters       ## For ChangeDetection()
  ${Boost_LIBRARIES}
)
