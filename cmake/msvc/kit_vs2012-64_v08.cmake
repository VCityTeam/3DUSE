# MSVC - KIT VS 2012 x64 - V08

if(DEFINED ENV{VCITY_KIT_ROOT})
	set( VCITY_KIT_ROOT $ENV{VCITY_KIT_ROOT} )
endif()
if( NOT DEFINED VCITY_KIT_ROOT )
	message(FATAL_ERROR "VCITY_KIT_ROOT not set.  Please set VCITY_KIT_ROOT.")
endif( NOT DEFINED VCITY_KIT_ROOT )

if(WITH_QT5)
	# with qt5
	set(QT5_DIR				${VCITY_KIT_ROOT}/Qt/Qt5.1.0/5.1.0/msvc2012_64_opengl)
else(WITH_QT5)
	# with qt4
	set(QTDIR				${VCITY_KIT_ROOT}/Qt/qt-4.8.5-x64-msvc2012)
endif(WITH_QT5)

set(ASSIMP_ROOT_DIR			${VCITY_KIT_ROOT}/assimp-3.0.1270)

set(OSG_DIR					${VCITY_KIT_ROOT}/osg/osg-3.2.1-bin-without-qt)

set(GDAL_INCLUDE_DIR		${OSG_DIR}/../3rdParty/x64/include)
set(GDAL_LIBRARY			${OSG_DIR}/../3rdParty/x64/lib/gdal_i.lib)

set(LIBXML2_INCLUDE_DIR		${OSG_DIR}/../3rdParty/x64/include/libxml2)
set(LIBXML2_LIBRARIES		${OSG_DIR}/../3rdParty/x64/lib/libxml2.lib) #libxml2D.lib

set(LASLIB_INCLUDE_DIR		${VCITY_KIT_ROOT}/laslib/include)
set(LASLIB_LIBRARY			${VCITY_KIT_ROOT}/laslib/lib/Release/laslib.lib)
set(LASLIB_LIBRARY_D		${VCITY_KIT_ROOT}/laslib/lib/Debug/laslib.lib)
