#include <string>

#include "gui/osg/osgScene.hpp"

struct Triangle
{
	Triangle(TVec3d a = TVec3d(0.0,0.0,0.0),TVec3d b = TVec3d(0.0,0.0,0.0),TVec3d c = TVec3d(0.0,0.0,0.0))
	{
		this->a = a;
		this->b = b;
		this->c = c;
		polygon = nullptr;
		geometry = nullptr;
		object = nullptr;
		parent = nullptr;
	}

	TVec3d a;
	TVec3d b;
	TVec3d c;

	citygml::Polygon* polygon;
	citygml::Geometry* geometry;
	citygml::CityObject* object;
	citygml::CityObject* parent;
};

struct Hit
{
	Hit()
	{
		intersect = false;
		triangle = false;
	}

	bool intersect;
	float distance;
	float parameter;
	float triangleBary[3];
	TVec3d point;
	Triangle* triangle;
};

struct Ray
{
	Ray(TVec3d ori = TVec3d(0.0,0.0,0.0),TVec3d dir = TVec3d(0.0,0.0,0.0))
	{
		this->ori = ori;
		this->dir = dir;
	}

	bool Intersect(Triangle* triangle, Hit* hit = nullptr);

	TVec3d ori;
	TVec3d dir;
};

struct GlobalData
{
	GlobalData()
	{
		minDistance = FLT_MAX;
		maxDistance = FLT_MIN;
	}

	float minDistance;
	float maxDistance;
};



void RayTracing(std::string path, TVec3d offset,osg::Camera* cam = nullptr);

std::vector<Triangle*> BuildTriangleList(std::string path, TVec3d offset);

/**
*	@brief Detecte les batiments à toits plats dans un fichier citygml, et écris un fichier shape
*	@param path Chemin vers le fichier citygml
*	@param minArea Aire minimum que doit avoir un toit plat pour être pris en compte
*	@param slopeFactor Factor permettant de définir à partir de quel pente on prend en compte le toit : 1 pas de pente, 0 pente verticale (valeur conseillé : 0.98) 
*/
void DetectionToitsPlats(std::string path, float minArea, float slopeFactor);