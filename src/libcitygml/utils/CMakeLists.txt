
add_library( citygmlutils SHARED
  # FIXME: compiled with a GLOB within the project CMakeLists.txt
  # FIXME AABB.cpp
  # FIXME Triangle.cpp
  ../export/exportJSON.cpp
  ../export/exportOBJ.cpp
  ../import/importerASC.cpp
  ../import/importerAssimp.cpp

  # FIXME: the following are dependencies of import/importerASC.cpp:
  #  1/ ToolAlgoCut provides the ProjectPointOnPolygon3D function
  #     (that itself depends on OGPolygon and thus OGR)
  #  2/ osgMnt.cpp is only used to import ASC files (actually this
  #     class should be renamed to ASC which is the extension of
  #     used for such files).
  ${CMAKE_SOURCE_DIR}/src/processes/ToolAlgoCut.cpp       # WTF
  ${CMAKE_SOURCE_DIR}/src/DataStructures/DEM/osgMnt.cpp   # WTF
)

target_include_directories( citygmlutils PUBLIC
  ${CITYGML_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}       # For e.g. src/DataStructures/DEM/osgMnt.hpp
  ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries( citygmlutils
  citygml
  ${ASSIMP_LIBRARY}
  ${OPENGL_glu_LIBRARY}
)
