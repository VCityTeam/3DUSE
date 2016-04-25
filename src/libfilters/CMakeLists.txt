include_directories(${CMAKE_SOURCE_DIR}/src  ${CMAKE_SOURCE_DIR}/src/libcitygml/utils)

file(
	GLOB_RECURSE
	source_files
	*.cpp
)

add_library(
	filters
	${source_files}
)

target_link_libraries(filters
  ${OSG_LIBRARIES}
  # FIXME citygmlutils
)

if (WITH_QT5)
#	QT5_USE_MODULES(filters Core Widgets OpenGL)
	QT5_USE_MODULES(filters Core)
endif(WITH_QT5)
