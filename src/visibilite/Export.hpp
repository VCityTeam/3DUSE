#ifndef __EXPORT_HPP__
#define __EXPORT_HPP__

#include "Visibilite.hpp"

/**
*	@brief Get the color of the sky
*	@param d Direction of the ray
*	@param light Direction of the light
*/
TVec3d SkyShadeBlue(TVec3d d, TVec3d light);

/**
*	@brief Process the results of a ray tracing algorithm
*	@param viewpoint Data about the viewpoint we are rendering
*	@param result Result from the ray tracing algorithm
*	@param filePrefix prefix to add before a file name
*/
void ExportData(ViewPoint* viewpoint, std::string filePrefix = "");

/**
*	@brief Export images from a ray tracing result
*	@param viewpoint Data about the viewpoint we are rendering
*	@param result Result from the ray tracing algorithm
*/
void ExportImages(ViewPoint* viewpoint, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the roof and wall of the buildings
*	@param viewpoint Data about the viewpoint we are rendering
*	@param result Result from the ray tracing algorithm
*/
void ExportImageRoofWall(ViewPoint* viewpoint, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the zBuffer of the scene
*	@param viewpoint Data about the viewpoint we are rendering
*	@param result Result from the ray tracing algorithm
*/
void ExportImageZBuffer(ViewPoint* viewpoint, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the objects of the scene with each a color
*	@param viewpoint Data about the viewpoint we are rendering
*	@param result Result from the ray tracing algorithm
*/
void ExportImageObjectColor(ViewPoint* viewpoint, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the remarquable building highlighted (the reste is in white
*	@param viewpoint Data about the viewpoint we are rendering
*	@param result Result from the ray tracing algorithm
*/
void ExportImageHighlightRemarquable(ViewPoint* viewpoint, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the skyline
*	@param viewpoint Data about the viewpoint we are rendering
*	@param result Result from the ray tracing algorithm
*/
void ExportImageSkyline(ViewPoint* viewpoint, std::string filePrefix = "");

/**
*	@brief Export image Test
*	@param viewpoint Data about the viewpoint we are rendering
*	@param result Result from the ray tracing algorithm
*/
void ExportImageTest(ViewPoint* viewpoint, std::string filePrefix = "");

#endif