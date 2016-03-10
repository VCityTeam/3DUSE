include_directories(${CMAKE_SOURCE_DIR}/src)

file(
	GLOB_RECURSE
	source_files
	*.cpp
)

add_library(
	citygmlutils
	${source_files}
)

if (WITH_QT5)
#	QT5_USE_MODULES(citygmlutils Core Widgets OpenGL)
	QT5_USE_MODULES(citygmlutils Core)
endif(WITH_QT5)
