#include "src/visibilite/VegetTool.hpp"

#include <qfiledialog.h>
#include "src/processes/ExportToShape.hpp"
#include "lasreader.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include "citygml.hpp"
#include "export/exportCityGML.hpp"

void CutShape(TVec2d min, TVec2d max, std::string outputFile)
{
	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load shp file");

	QFileInfo file(filepath);

	QString ext = file.suffix().toLower();

	if(ext == "shp")
	{
		citygml::CityModel* ModelOut = new citygml::CityModel;

		OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), TRUE);
		std::cout << "Shp loaded" << std::endl;
		std::cout << "Processing..." << std::endl;

		OGRLayer *poLayer;
		int nbLayers = poDS->GetLayerCount();
		if(nbLayers > 0)
		{
			OGRMultiPolygon* vegetOuput = new OGRMultiPolygon;

			poLayer = poDS->GetLayer(0);

			OGRFeature *poFeature;
			poLayer->ResetReading();

			unsigned int cpt = 0;

			while( (poFeature = poLayer->GetNextFeature()) != NULL )
			{
				OGRGeometry* poGeometry = poFeature->GetGeometryRef();

				if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
				{

					//Emprise au sol
					OGRPolygon* poPG = (OGRPolygon*) poGeometry;
					OGRLinearRing* poLR = poPG->getExteriorRing();
					bool found = false;

					for(unsigned int i = 0; i < poLR->getNumPoints(); i++)
					{
						OGRPoint p;
						poLR->getPoint(i, &p);
						if(p.getX() >= min.x && p.getX() <= max.x && p.getY() >= min.y && p.getY() <= max.y)
						{
							found = true;
							break;
						}
					}

					if(found)
					{
						vegetOuput->addGeometryDirectly(poPG);
					}
				}
			}

			SaveGeometrytoShape("SkylineOutput/"+outputFile, vegetOuput);
			delete vegetOuput;
		}
	}

	std::cout << "Done !" << std::endl;
}

void ProcessLasShpVeget()
{
	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load Shp file");

	QFileInfo file(filepath);

	QString ext = file.suffix().toLower();

	std::cout << "Shp : " << filepath.toStdString() << std::endl;

	if(ext == "shp")
	{

		QString filepathBis = QFileDialog::getOpenFileName(nullptr,"Load Las file");

		QFileInfo fileBis(filepathBis);

		QString extBis = fileBis.suffix().toLower();

		std::cout << "Las : " << filepathBis.toStdString() << std::endl;

		if(extBis == "las")
		{
			OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), TRUE);
			std::cout << "Shp loaded" << std::endl;

			LASreadOpener lasreadopener;
			lasreadopener.set_file_name(filepathBis.toStdString().c_str());
			LASreader* lasreader = lasreadopener.open();
			std::cout << "Las loaded" << std::endl;

			std::vector<OGRPolygon*> vegetPoly;//Where we are going to store all polygon

			//We are going to read the shp and store all polygon to a vector
			//=======<ShpReading>========
			OGRLayer *poLayer;
			int nbLayers = poDS->GetLayerCount();
			if(nbLayers > 0)
			{
				OGRMultiPolygon* vegetOuput = new OGRMultiPolygon;

				poLayer = poDS->GetLayer(0);

				OGRFeature *poFeature;
				poLayer->ResetReading();

				unsigned int cpt = 0;

				while( (poFeature = poLayer->GetNextFeature()) != NULL )
				{
					OGRGeometry* poGeometry = poFeature->GetGeometryRef();

					if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
					{
						//Emprise au sol
						OGRPolygon* poPG = (OGRPolygon*) poGeometry;
						vegetPoly.push_back(poPG);
					}
				}
			}
			//=======<ShpReading>========

			std::cout << "Veget Poly Vector Done." << std::endl;
			std::cout << "Processing LAS." << std::endl;

			std::map<OGRPolygon*,std::vector<OGRPoint>> points;
			OGRMultiPoint* mp = new OGRMultiPoint;

			//For every point
			while (lasreader->read_point())
			{
				OGRPoint* ptemp = new OGRPoint((lasreader->point).get_x(),(lasreader->point).get_y(),(lasreader->point).get_z());
				bool found = false;
				for(unsigned int i = 0; i < vegetPoly.size(); i++)//For every poly
				{
					if(vegetPoly[i]->Contains(ptemp))//Checking if the point is in the poly
					{
						mp->addGeometry(ptemp);
						points[vegetPoly[i]].push_back(*ptemp);
						found = true;
						break;
					}
				}
				if(!found)
					delete ptemp;
			}

			std::cout << "Done." << std::endl;
			std::cout << "Exporting." << std::endl;

			ProcessCL(points,fileBis.baseName().toStdString());

			std::filebuf fb;
			fb.open("SkylineOutput/"+fileBis.baseName().toStdString()+".dat",std::ios::out);

			std::ostream file(&fb);

			file << points.size() << "\n";

			for(auto it = points.begin(); it != points.end(); it++)
			{
				file << it->second.size() << "\n";
				for(OGRPoint p : it->second)
				{
					file << std::fixed << p.getX() << "\n";
					file << std::fixed << p.getY() << "\n";
					file << std::fixed << p.getZ() << "\n";
				}
			}

			fb.close();

			std::cout << "Done All." << std::endl;

		}

	}
}

citygml::CityObject* VegToCityObject(std::vector<OGRPoint> veg, std::string outputFile)
{
	static int cpt = 0;
	std::string name = "veget_"+std::to_string(cpt);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	for(unsigned int i = 0; i < veg.size(); i++)
	{
		cloud->push_back(pcl::PointXYZ(veg[i].getX(),veg[i].getY(),veg[i].getZ()));
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
	gp3.setSearchRadius (0.025);

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
}

void ProcessCL(std::map<OGRPolygon*,std::vector<OGRPoint>> vegets, std::string outputFile)
{
	std::cout << "Trying to process" << std::endl;

	citygml::CityModel* ModelOut = new citygml::CityModel;

	std::cout << "To process : " << vegets.size() << std::endl;

	unsigned int i = 0;

	for(auto it = vegets.begin(); it != vegets.end(); it++)
	{
		std::cout << "Processing : " << i++ << std::endl;
		citygml::CityObject* BuildingCO = VegToCityObject(it->second,outputFile);
		ModelOut->addCityObject(BuildingCO);
		ModelOut->addCityObjectAsRoot(BuildingCO);
	}

	citygml::ExporterCityGML exporter("./SkylineOutput/"+outputFile+".gml");
	exporter.exportCityModel(*ModelOut);

	std::cout << "It is done !" << std::endl;
}