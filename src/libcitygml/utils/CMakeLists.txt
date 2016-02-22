include_directories(${CMAKE_SOURCE_DIR}/src)

file(
	GLOB_RECURSE
	source_files
	*.cpp
)

add_library(
	libcitygmlutils
	${source_files}
)
