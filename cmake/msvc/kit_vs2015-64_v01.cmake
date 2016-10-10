# MSVC - KIT VS 2015 x64 - V01

if(DEFINED ENV{VCITY_KIT_ROOT})
  set( VCITY_KIT_ROOT $ENV{VCITY_KIT_ROOT} )
endif()
if( NOT DEFINED VCITY_KIT_ROOT )
  message(FATAL_ERROR "VCITY_KIT_ROOT not set.  Please set VCITY_KIT_ROOT.")
endif()

if( BUILD_GUI_QT5 )
  set(QT5_DIR           ${VCITY_KIT_ROOT}/Qt/Qt5.6.0/5.6/msvc2015_64)
endif()
if( BUILD_GUI_QT4 )
  set(QTDIR             ${VCITY_KIT_ROOT}/Qt/qt-4.8.7-x64-msvc2015)
endif()

set(ASSIMP_ROOT_DIR     ${VCITY_KIT_ROOT}/assimp-3.2)

set(GDAL_INCLUDE_DIR    ${VCITY_KIT_ROOT}/gdal/gdal-1.11.4/include)
set(GDAL_LIBRARY        ${VCITY_KIT_ROOT}/gdal/gdal-1.11.4/lib/gdal_i.lib)

set(LASLIB_INCLUDE_DIR  ${VCITY_KIT_ROOT}/laslib/include)
set(LASLIB_LIBRARY      ${VCITY_KIT_ROOT}/laslib/lib/Release/laslib.lib)
set(LASLIB_LIBRARY_D    ${VCITY_KIT_ROOT}/laslib/lib/Debug/laslib.lib)

set(LIBXML2_INCLUDE_DIR ${VCITY_KIT_ROOT}/libxml2-2.9.3/include/libxml2)
set(LIBXML2_LIBRARIES   ${VCITY_KIT_ROOT}/libxml2-2.9.3/lib/libxml2.lib)

set(OSG_DIR             ${VCITY_KIT_ROOT}/osg/OpenSceneGraph-3.4.0)

###### PCL and its sub-dependencies
set(PCL_DIR             ${MSVC_KIT_ROOT}/PCL/pcl-1.7.2/cmake)
set(BOOST_ROOT          ${MSVC_KIT_ROOT}/boost_1_59_0)
set(EIGEN3_INCLUDE_DIR  ${MSVC_KIT_ROOT}/eigen-3.2.8)
