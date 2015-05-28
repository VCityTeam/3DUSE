#ifndef __EXPORT_HPP__
#define __EXPORT_HPP__

#include "Visibilite.hpp"

/**
*	@brief Process the results of a ray tracing algorithm
*	@param globalData Where to write some data generate globaly
*	@param result Result from the ray tracing algorithm
*	@param filePrefix prefix to add before a file name
*/
void ExportData(GlobalData* globalData, RayTracingResult* result, std::string filePrefix = "");

/**
*	@brief Export images from a ray tracing result
*	@param globalData Where to write some data generate globaly
*	@param result Result from the ray tracing algorithm
*/
void ExportImages(GlobalData* globalData, RayTracingResult* result, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the roof and wall of the buildings
*	@param globalData Where to write some data generate globaly
*	@param result Result from the ray tracing algorithm
*/
void ExportImageRoofWall(GlobalData* globalData, RayTracingResult* result, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the zBuffer of the scene
*	@param globalData Where to write some data generate globaly
*	@param result Result from the ray tracing algorithm
*/
void ExportImageZBuffer(GlobalData* globalData, RayTracingResult* result, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the objects of the scene with each a color
*	@param globalData Where to write some data generate globaly
*	@param result Result from the ray tracing algorithm
*/
void ExportImageObjectColor(GlobalData* globalData, RayTracingResult* result, std::string filePrefix = "");

/**
*	@brief Export image reprensenting the remarquable building highlighted (the reste is in white
*	@param globalData Where to write some data generate globaly
*	@param result Result from the ray tracing algorithm
*/
void ExportImageHighlightRemarquable(GlobalData* globalData, RayTracingResult* result, std::string filePrefix = "");

#endif