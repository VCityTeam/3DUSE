include_directories(${CMAKE_SOURCE_DIR}/src  ${CMAKE_SOURCE_DIR}/src/libcitygml/utils)

file(
	GLOB_RECURSE
	source_files
	*.cpp
)

add_library(
	raytracing
	${source_files}
)

target_link_libraries(raytracing ${OSG_LIBRARIES} libcitygmlutils)

