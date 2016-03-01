#ifndef __VIEWPOINT_HPP__
#define __VIEWPOINT_HPP__

#include <qcolor.h>

#include "Hit.hpp"

struct Skyline
{
public:
	std::vector<std::pair<unsigned int,unsigned int>> FragCoords;
	std::vector<TVec3d> Points;
	TVec3d Position;

	std::vector<double> Radius;
	double RadiusMin;
	double RadiusMax;
	double Average;
	double StandardDeviation;

	void ComputeData();
};


/**
*	@brief Information about a viewpoint in the city
*/
struct ViewPoint
{
	/**
	*	Build a new viewpoint
	*	@param width Horizontal resolution of the viewpoint
	*	@param height Vertical resolution of the viewpoint
	*/
    ViewPoint(unsigned int width, unsigned int height, std::string id = "")
	{
        this->id = id;
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
    std::string id; ///< Id of the viewpoint, useful for DoMonoTileAnalysis
	unsigned int width;///< Width of hits
	unsigned int height;///< Height of hits
	TVec3d lightDir;///< Direction of the light in the scene
	float minDistance;///< Minimum distance to a triangle in the scene
	float maxDistance;///< Maximum distance to a triangle in the scene
	Skyline skyline;///< Skyline of the viewpoints
	TVec3d position;

private:
	/**
	*	Clamp x between a and b
	*/
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

#endif
