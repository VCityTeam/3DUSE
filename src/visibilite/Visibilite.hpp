#ifndef __VISIBILITE_HPP__
#define __VISIBILITE_HPP__

#include <string>
#include <unordered_map>

#include <qrgb.h>
#include <QColor>

#include "Triangle.hpp"
#include "Hit.hpp"

struct ViewPoint
{
	ViewPoint(unsigned int width, unsigned int height)
	{
		this->width = width;
		this->height = height;
		hits = new Hit*[width];
		for(unsigned int i = 0; i < width; i++)
			hits[i] = new Hit[height];

		minDistance = FLT_MAX;
		maxDistance = FLT_MIN;
	}

	ViewPoint()
	{
		minDistance = FLT_MAX;
		maxDistance = FLT_MIN;
		hits = nullptr;
	}

	~ViewPoint()
	{
		if(hits != nullptr)
		{
			for(unsigned int i = 0; i < width; i++)
				delete[] hits[i];
			delete[] hits;
		}
	}

	/**
	*	@brief Compute the skyline of the viewpoint
	*/
	void ComputeSkyline();

	/**
	*	@brief Reset the viewpoint hits array
	*/
	void Reset();

	/**
	*	@brief Compute the min and max distance
	*/
	void ComputeMinMaxDistance();

	Hit** hits;///< Hit of the rays
	unsigned int width;///< Width of hits
	unsigned int height;///< Height of hits
	TVec3d lightDir;///< Direction of the light in the scene
	float minDistance;///< Minimum distance to a triangle in the scene
	float maxDistance;///< Maximum distance to a triangle in the scene
	std::map<std::string,QColor> objectToColor;///< Map a city object id to a color
	std::vector<std::pair<unsigned int,unsigned int>> skyline;///< Skyline of the viewpoints

private:
	inline unsigned int Clamp(unsigned int x,unsigned int a,unsigned int b);

	/**
	*	Enum used when build skyline, position relative to a pixel
	*/
	enum Position
	{
		W = 0,//West
		NW = 1,//North West
		N = 2,//North
		NE = 3,//North East
		E = 4,//East
		SE = 5,//South East
		S = 6,//South
		SW = 7//South West
	};

	/**
	*	@brief Get the shift need to know the coordinate in the position p
	*	@param p Position
	*	@return Shift need in <x,y>
	*/
	std::pair<int, int> GetCoord(Position p);
};

struct RayCollection
{
	RayCollection(std::vector<Ray*> rays = std::vector<Ray*>())
	{
		this->rays = rays;
	}

	~RayCollection()
	{
		for(unsigned int i = 0; i < rays.size(); i++)
		{
			delete rays[i];
		}
	}

	static RayCollection* BuildCollection(osg::Camera* cam);

	std::vector<Ray*> rays;
	ViewPoint* viewpoint;///< The viewpoint render using this collection
};

struct AnalysisResult
{
	std::vector<TVec3d> skyline;
	TVec3d viewpointPosition;
	ViewPoint* viewpoint;
};

std::vector<AnalysisResult> Analyse(std::string dirTile, TVec3d offset, osg::Camera* cam, std::string prefix = "");
std::vector<AnalysisResult> Analyse(std::string dirTile, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement);
std::vector<AnalysisResult> Analyse(std::string dirTile, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints);
std::vector<AnalysisResult> AnalysePanorama(std::string dirTile, TVec3d offset,osg::Camera* cam);

/**
*	@brief Perform a viewpoint analysis on different viewpoint
*	@param cams List of camera corresponding to viewpoints
*	@param buildingPath Path to the citygml file of the buildings
*	@param terrainPath Path to the citygml file of the terrains
*	@param offset Offset of the geometry in 3Duse
*/
std::vector<AnalysisResult> DoMonoTileAnalysis(std::vector<osg::ref_ptr<osg::Camera>> cams,std::vector<std::string> paths, TVec3d offset);


std::vector<AnalysisResult> AnalyseTestCam(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam);

/**
*	@brief Prep camera for a single viewpoint analysis
*	@param buildingPath Path to the citygml file of the buildings
*	@param terrainPath Path to the citygml file of the terrains
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*/
std::vector<AnalysisResult> Analyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam);

/**
*	@brief Prep camera for cascade viewpoints
*	@param buildingPath Path to the citygml file of the buildings
*	@param terrainPath Path to the citygml file of the terrains
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*	@param count How many time we rune the analysis
*	@param zIncrement How much we increase the z coordinate of the cam each analysis
*/
std::vector<AnalysisResult> Analyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement);

/**
*	@brief Prep camera for a list of viewpoint
*	@param buildingPath Path to the citygml file of the buildings
*	@param terrainPath Path to the citygml file of the terrains
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*	@param viewpoints List of viewpoint where to analyse, <position,target>
*/
std::vector<AnalysisResult> Analyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints);


std::vector<AnalysisResult> AnalyseTestViewport(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam);

/**
*	@build Perform a raytracing on a set of triangles
*	@param triangles The list of triangle from the CityGML tile
*	@param rays List of rays
*/
void RayTracing(TriangleList* triangles, std::vector<Ray*> rays);

/**
*	@brief Build list of triangle from a CityGML building tile
*	@param tile CityGML tile from which we want the triangle list
*	@param offset offset Offset of the geometry in 3Duse
*	@param viewpoint Data about the viewpoint we are rendering
*	@param ignoreMiscBuilding If true this will ignore building that are not remarquable
*	@return The list of triangle from the CityGML tile
*/
TriangleList* BuildTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint, citygml::CityObjectsType objectType);

/**
*	@brief Build the osg node to display a skyline in 3DUse
*	@param skyline Skyline
*	@return The osg node
*/
osg::ref_ptr<osg::Geode> BuildSkylineOSGNode(std::vector<TVec3d> skyline);

/**
*	@Brief Build the viewshed
*	@param result Result of the analysis
*/
osg::ref_ptr<osg::Geode> BuildViewshedOSGNode(AnalysisResult result);

#endif