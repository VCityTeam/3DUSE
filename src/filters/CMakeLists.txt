add_subdirectory( Test )

add_library( filters SHARED
  ChangeDetection/ChangeDetection.cpp
  EnhanceCityGML/EnhanceMNT.cpp
  EnhanceCityGML/LinkCityGMLShape.cpp
  raytracing/Hit.cpp
  raytracing/Ray.cpp
  raytracing/RayTracing.cpp
  ShpExtrusion/ShpExtrusion.cpp
  Tiling/ASCCut.cpp
  Tiling/TilingCityGML.cpp
)

target_include_directories( filters PUBLIC
  ${VCITYCORE_INCLUDE_DIR}
  ${CITYGMLUTILS_INCLUDE_DIR}
)

target_link_libraries(filters
  ${VCITYCORE_LIBRARY}
  ${CITYGMLUTILS_LIBRARIES}
)

if( WITH_QT5 )
  target_include_directories( filters PRIVATE
    $<TARGET_PROPERTY:Qt5::Core,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries( filters Qt5::Core )
endif()

GENERATE_EXPORT_HEADER( filters )
