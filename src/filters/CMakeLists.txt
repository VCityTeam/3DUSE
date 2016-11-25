add_subdirectory( Test )

add_library( filters SHARED
  ChangeDetection/ChangeDetection.cpp
  EnhanceCityGML/EnhanceMNT.cpp
  EnhanceCityGML/LinkCityGMLShape.cpp
  ExtractRoof/ExtractRoof.cpp
  raytracing/Hit.cpp
  raytracing/Ray.cpp
  raytracing/RayTracing.cpp
  ShpExtrusion/ShpExtrusion.cpp
  Tiling/ASCCut.cpp
  Tiling/TilingCityGML.cpp
)

if(MSVC)
  # The build dll goes to the top level build directory.
  # The associated import library goes...out of the way.
  foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )  # Just in case
    set_target_properties( filters PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}
      LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
         ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG}/Lib
    )
  endforeach()
endif()

target_include_directories( filters PUBLIC
  ${VCITYCORE_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

target_link_libraries(filters
  ${VCITYCORE_LIBRARY}
  ${CITYGMLUTILS_LIBRARIES}
)

GENERATE_EXPORT_HEADER( filters )
