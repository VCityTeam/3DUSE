#include "src/visibilite/ProcessCloudPoint.h"

#include "src/core/application.hpp"

#include <string>
#include <fstream>
#include <vector>
/*
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>*/

#include "citygml.hpp"
#include "export/exportCityGML.hpp"

/*citygml::CityObject* VegToCityObject(std::vector<TVec3d> veg, unsigned int cpt)
{
	std::string name = "veget_"+std::to_string(cpt);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	for(unsigned int i = 0; i < veg.size(); i++)
	{
		cloud->push_back(pcl::PointXYZ(veg[i].x,veg[i].y,veg[i].z));
	}

	//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (1);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	citygml::CityObject* BuildingCO = new citygml::SolitaryVegetationObject(name);
	//BuildingCO

	//pcl::PointCloud<pcl::PointXYZ> resultcloud; 
	//cloud.points[ mesh.polygons[tri_i].vertices[vertex_i] ] 

	pcl::PointCloud<pcl::PointXYZ> cloudResult;
	pcl::fromPCLPointCloud2(triangles.cloud, cloudResult);

	citygml::Geometry* cityGeom = new citygml::Geometry(name+"_Geometry", citygml::GT_Unknown, 2);

	if(triangles.polygons.size() > 0)
		std::cout << "Awww " << cpt << std::endl;

	for(unsigned int i = 0; i < triangles.polygons.size(); i++)
	{
		pcl::Vertices polys = triangles.polygons[i];
		for(unsigned int j = 0; j < polys.vertices.size(); j += 3)
		{
			pcl::PointXYZ a = cloudResult[polys.vertices[j]];
			pcl::PointXYZ b = cloudResult[polys.vertices[j+1]];
			pcl::PointXYZ c = cloudResult[polys.vertices[j+2]];

			citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(i)+std::to_string(j)+"_ring",true);
			ring->addVertex(TVec3d(a.x,a.y,a.z));
			ring->addVertex(TVec3d(b.x,b.y,b.z));
			ring->addVertex(TVec3d(c.x,c.y,c.z));
			citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(i)+std::to_string(j)+"_poly");
			poly->addRing(ring);
			cityGeom->addPolygon(poly);
		}
	}

	BuildingCO->addGeometry(cityGeom);

	return BuildingCO;
}*/

citygml::CityObject* VegToCityObject(std::vector<TVec3d> veg, unsigned int cpt)
{
	std::string name = "veget_"+std::to_string(cpt);
	unsigned int ii = 0;
	unsigned int jj = 0;

	double xmin = DBL_MAX;
	double ymin = DBL_MAX;
	double zmin = DBL_MAX;
	double xmax = -DBL_MAX;
	double ymax = -DBL_MAX;

	double average = 0.0;

	for(TVec3d p : veg)
	{
		xmin = std::min(xmin,p.x);
		ymin = std::min(ymin,p.y);
		zmin = std::min(zmin,p.z);
		xmax = std::max(xmax,p.x);
		ymax = std::max(ymax,p.y);
		average += p.z;
	}

	average /= veg.size();

	xmin = floor(xmin);
	ymin = floor(ymin);
	xmax = ceil(xmax);
	ymax = ceil(ymax);

	unsigned int xi = static_cast<unsigned int>(xmin);
	unsigned int yi = static_cast<unsigned int>(ymin);
	unsigned int xa = static_cast<unsigned int>(xmax);
	unsigned int ya = static_cast<unsigned int>(ymax);

	unsigned int width = xa - xi;
	unsigned int height = ya - yi;

	double** table = new double*[width];
	for(unsigned int i = 0; i < width; i++)
	{
		table[i] = new double[height];
		for(unsigned int j = 0; j < height; j++)
		{
			table[i][j] = 0.0;
		}
	}


	for(TVec3d p : veg)
	{
		unsigned int xt = static_cast<unsigned int>(p.x) - xi;
		unsigned int yt = static_cast<unsigned int>(p.y) - yi;

		if(xt < width && yt < height)
		{
			table[xt][yt] = p.z;
		}
	}


	citygml::CityObject* BuildingCO = new citygml::SolitaryVegetationObject(name);

	citygml::Geometry* cityGeom = new citygml::Geometry(name+"_Geometry", citygml::GT_Unknown, 2);

	unsigned int factor = 2;

	if(width > factor && height > factor)
	{
		for(unsigned int i = 0; i < width-factor; i+=factor)
		{
			for(unsigned int j = 0; j < height-factor; j+=factor)
			{
				if(table[i][j] > 0.0 && table[i+factor][j] > 0.0 && table[i+factor][j+factor] > 0.0 && table[i][j+factor] > 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(i+xi,j+yi,table[i][j]));
					ring->addVertex(TVec3d(i+factor+xi,j+yi,table[i+factor][j]));
					ring->addVertex(TVec3d(i+factor+xi,j+factor+yi,table[i+factor][j+factor]));
					ring->addVertex(TVec3d(i+xi,j+factor+yi,table[i][j+factor]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
				else if(table[i][j] > 0.0 && table[i+factor][j] > 0.0 && table[i+factor][j+factor] > 0.0 && table[i][j+factor] == 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(i+xi,j+yi,table[i][j]));
					ring->addVertex(TVec3d(i+factor+xi,j+yi,table[i+factor][j]));
					ring->addVertex(TVec3d(i+factor+xi,j+factor+yi,table[i+factor][j+factor]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
				else if(table[i][j] > 0.0 && table[i+factor][j] > 0.0 && table[i+factor][j+factor] == 0.0 && table[i][j+factor] > 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(i+xi,j+yi,table[i][j]));
					ring->addVertex(TVec3d(i+factor+xi,j+yi,table[i+factor][j]));
					ring->addVertex(TVec3d(i+xi,j+factor+yi,table[i][j+factor]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
				else if(table[i][j] > 0.0 && table[i+factor][j] == 0.0 && table[i+factor][j+factor] > 0.0 && table[i][j+factor] > 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(i+xi,j+yi,table[i][j]));
					ring->addVertex(TVec3d(i+factor+xi,j+factor+yi,table[i+factor][j+factor]));
					ring->addVertex(TVec3d(i+xi,j+factor+yi,table[i][j+factor]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
				else if(table[i][j] == 0.0 && table[i+factor][j] > 0.0 && table[i+factor][j+factor] > 0.0 && table[i][j+factor] > 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(i+factor+xi,j+yi,table[i+factor][j]));
					ring->addVertex(TVec3d(i+factor+xi,j+factor+yi,table[i+factor][j+factor]));
					ring->addVertex(TVec3d(i+xi,j+factor+yi,table[i][j+factor]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
			}
		}
	}

	/*{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(i++)+std::to_string(j++)+"_ring",true);
	ring->addVertex(TVec3d(xmin,ymin,zmin));
	ring->addVertex(TVec3d(xmin,ymax,zmin));
	ring->addVertex(TVec3d(xmax,ymax,zmin));
	ring->addVertex(TVec3d(xmax,ymin,zmin));
	citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(i++)+std::to_string(j++)+"_poly");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}
	{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(i++)+std::to_string(j++)+"_ring",true);
	ring->addVertex(TVec3d(xmin,ymin,average));
	ring->addVertex(TVec3d(xmin,ymax,average));
	ring->addVertex(TVec3d(xmax,ymax,average));
	ring->addVertex(TVec3d(xmax,ymin,average));
	citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(i++)+std::to_string(j++)+"_poly");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}*/

	/*{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(i++)+std::to_string(j++)+"_ring",true);
	ring->addVertex(TVec3d(xmin,ymin,average));
	ring->addVertex(TVec3d(xmin,ymax,average));
	ring->addVertex(TVec3d(xmax,ymax,average));
	ring->addVertex(TVec3d(xmax,ymin,average));
	citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(i++)+std::to_string(j++)+"_poly");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}*/

	/*for(unsigned int i = 0; i < triangles.polygons.size(); i++)
	{
		pcl::Vertices polys = triangles.polygons[i];
		for(unsigned int j = 0; j < polys.vertices.size(); j += 3)
		{
			pcl::PointXYZ a = cloudResult[polys.vertices[j]];
			pcl::PointXYZ b = cloudResult[polys.vertices[j+1]];
			pcl::PointXYZ c = cloudResult[polys.vertices[j+2]];

			citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(i)+std::to_string(j)+"_ring",true);
			ring->addVertex(TVec3d(a.x,a.y,a.z));
			ring->addVertex(TVec3d(b.x,b.y,b.z));
			ring->addVertex(TVec3d(c.x,c.y,c.z));
			citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(i)+std::to_string(j)+"_poly");
			poly->addRing(ring);
			cityGeom->addPolygon(poly);
		}
	}*/

	BuildingCO->addGeometry(cityGeom);

	return BuildingCO;
}

void ProcessCL()
{
	std::string path = "C:/VCityBuild/SkylineOutput/1842_5176.dat";

	char line[256];

	std::ifstream ifs (path, std::ifstream::in);

	ifs.getline(line,256);

	unsigned int count = atoi(line);

	std::vector<std::vector<TVec3d>> vegets;

	for(unsigned int i = 0; i < count; i++)
	{
		std::vector<TVec3d> veg;
		ifs.getline(line,256);
		unsigned int countBis = atoi(line);

		for(unsigned int j = 0; j < countBis; j++)
		{
			double x;
			double y;
			double z;
			
			ifs.getline(line,256);x = atof(line);
			ifs.getline(line,256);y = atof(line);
			ifs.getline(line,256);z = atof(line);

			veg.push_back(TVec3d(x,y,z));
		}

		vegets.push_back(veg);
	}

	ifs.close();

	std::cout << "Trying to process" << std::endl;

	citygml::CityModel* ModelOut = new citygml::CityModel;

	std::cout << "To process : " << vegets.size() << std::endl;

	for(unsigned int i = 0; i < vegets.size(); i++)
	{
		std::cout << "Processing : " << i << std::endl;
		citygml::CityObject* BuildingCO = VegToCityObject(vegets[i],i);
		ModelOut->addCityObject(BuildingCO);
		ModelOut->addCityObjectAsRoot(BuildingCO);
	}

	citygml::ExporterCityGML exporter("./SkylineOutput/1842_5176.gml");
	exporter.exportCityModel(*ModelOut);

	std::cout << "It is done !" << std::endl;
}