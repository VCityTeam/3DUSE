add_executable(testSunlightComputation testSunlightComputation.cxx)
target_link_libraries( testSunlightComputation VCityGui )

add_test( SUNLIGHT_COMPUTATION
          testSunlightComputation
        )


