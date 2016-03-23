// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __ALGO_HPP__
#define __ALGO_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <string>
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
class Algo
{
public:
	Algo();
    ~Algo();

	void CompareTwoLidar(std::string Path1, std::string Path2);

	/*void ConvertLasToPCD();
	void ExtractGround();
	void ExtractBuildings();
	void RemoveGroundWithTIN();
	void PrepareTIN();
	void CompareBuildings();
	void ConstructRoofs();*/

private:
};
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __ALGO_HPP__



	////////////////////////////////////////////////////////////////////////////////
	/*void Algo::ConvertLasToPCD()
	{
		LASreadOpener lasreadopener;
		//lasreadopener.set_file_name("C://Users//FredLiris//Downloads//1845_5181.las");
		lasreadopener.set_file_name("C://Users//FredLiris//Downloads//Grand Lyon LiDAR//Grand Lyon CHANGEMENTS CRAPONNE//LAS 2015//1833_5173_2015.las");
		LASreader* lasreader = lasreadopener.open();

		pcl::PointXYZ point;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		while (lasreader->read_point())
		{
			point.x = (lasreader->point).get_x();
			point.y = (lasreader->point).get_y();
			point.z = (lasreader->point).get_z();

			cloud->points.push_back(point);
		}

		cloud->width = (int)cloud->points.size();
		cloud->height = 1;
		cloud->is_dense = false;

		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("1845_5181.pcd", *cloud, false);

		std::cout << "Fichier PCD cree." << std::endl;
	}

	void Algo::ExtractGround()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndicesPtr ground (new pcl::PointIndices);

		// Fill in the cloud data
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZ> ("C://Users//FredLiris//Downloads//1845_5181.pcd", *cloud);

		std::cerr << "Cloud before filtering: " << std::endl;
		std::cerr << *cloud << std::endl;

		// Create the filtering object
		pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
		pmf.setInputCloud (cloud);
		pmf.setMaxWindowSize (20);
		pmf.setSlope (1.0f);
		pmf.setInitialDistance (0.5f);
		pmf.setMaxDistance (3.0f);
		pmf.extract (ground->indices);

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (ground);
		extract.filter (*cloud_filtered);

		std::cerr << "Ground cloud after filtering: " << std::endl;
		std::cerr << *cloud_filtered << std::endl;

		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("1845_5181_ground.pcd", *cloud_filtered, false);

		// Extract non-ground returns
		extract.setNegative (true);
		extract.filter (*cloud_filtered);

		std::cerr << "Object cloud after filtering: " << std::endl;
		std::cerr << *cloud_filtered << std::endl;

		writer.write<pcl::PointXYZ> ("1845_5181_object.pcd", *cloud_filtered, false);
	}

	void Algo::ExtractBuildings()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>), cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);;

		// Fill in the cloud data
		pcl::PCDReader reader;
		pcl::PCDWriter writer;

		reader.read<pcl::PointXYZ> ("C://Users//FredLiris//Downloads//1845_5181_NoGround_2m.pcd", *cloud);

		std::cerr << "PointCloud : " << cloud->width * cloud->height << " data points." << std::endl;

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cloud_trie = SegmentationRoofs(cloud);
	}

	void Algo::PrepareTIN()
	{
		std::string TINPath = "D:/Donnees/Data/Donnees_Sathonay/SATHONAY_CAMP_MNT_2012.gml";
		std::cout << "load citygml file : " << TINPath << std::endl;
		vcity::Tile* tile = new vcity::Tile(TINPath);

		citygml::CityModel* model = tile->getCityModel();

		OGRMultiPolygon* TinTerrain = new OGRMultiPolygon;

		for(citygml::CityObject* obj : model->getCityObjectsRoots())
		{
			if(obj->getType() == citygml::COT_TINRelief)
			{
				for(citygml::Geometry* Geometry : obj->getGeometries())
				{
					for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
					{
						OGRPolygon * OgrPoly = new OGRPolygon;
						OGRLinearRing * OgrRing = new OGRLinearRing;

						for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
						{
							OgrRing->addPoint(Point.x, Point.y, Point.z);
						}

						OgrRing->closeRings();

						if(OgrRing->getNumPoints() > 3)
						{
							OgrPoly->addRingDirectly(OgrRing);
							if(OgrPoly->IsValid())
								TinTerrain->addGeometryDirectly(OgrPoly);
						}
						else
							delete OgrRing;
					}
				}
			}
		}

		delete tile;
		SaveGeometrytoShape("TIN.shp", TinTerrain);

		OGRGeometry* Envelope = new OGRPolygon;

		for(int i = 0; i < TinTerrain->getNumGeometries(); ++i)
		{
			OGRGeometry* tmp = Envelope;
			Envelope = tmp->Union(TinTerrain->getGeometryRef(i));
			delete tmp;
		}

		SaveGeometrytoShape("TIN_Envelope.shp", Envelope);

		delete TinTerrain;
		delete Envelope;
	}

	void Algo::RemoveGroundWithTIN()
	{		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZ> ("C:/Users/FredLiris/Downloads/1845_5181.pcd", *cloud);
		std::cerr << "Cloud : " << std::endl;
		std::cerr << *cloud << std::endl;

		OGREnvelope* CloudBBox = new OGREnvelope;

		CloudBBox->MinX = cloud->at(0).x;
		CloudBBox->MaxX = cloud->at(0).x;
		CloudBBox->MinY = cloud->at(0).y;
		CloudBBox->MaxY = cloud->at(0).y;

		for(pcl::PointXYZ P : cloud->points)
		{
			if(P.x > CloudBBox->MaxX)
				CloudBBox->MaxX = P.x;
			if(P.x < CloudBBox->MinX)
				CloudBBox->MinX = P.x;
			if(P.y > CloudBBox->MaxY)
				CloudBBox->MaxY = P.y;
			if(P.y < CloudBBox->MinY)
				CloudBBox->MinY = P.y;
		}

		OGRLinearRing * Ring = new OGRLinearRing;
		Ring->addPoint(CloudBBox->MinX, CloudBBox->MinY);
		Ring->addPoint(CloudBBox->MinX, CloudBBox->MaxY);
		Ring->addPoint(CloudBBox->MaxX, CloudBBox->MaxY);
		Ring->addPoint(CloudBBox->MaxX, CloudBBox->MinY);
		Ring->addPoint(CloudBBox->MinX, CloudBBox->MinY);
		OGRPolygon* PolyBBox = new OGRPolygon;
		PolyBBox->addRingDirectly(Ring);
		delete CloudBBox;

		SaveGeometrytoShape("BoundingBox_1845_5181.shp", PolyBBox);

		OGRMultiPolygon* TinTerrain = new OGRMultiPolygon;
		OGRMultiPolygon* TinTerrainEnvelope = new OGRMultiPolygon;

		OGRDataSource* DS_TinTerrain = OGRSFDriverRegistrar::Open("TIN.shp", TRUE);
		OGRDataSource* DS_TinTerrainEnvelope = OGRSFDriverRegistrar::Open("TIN_Envelope.shp", TRUE);
		OGRLayer* L_TinTerrain = DS_TinTerrain->GetLayer(0);
		OGRLayer* L_TinTerrainEnvelope = DS_TinTerrainEnvelope->GetLayer(0);

		OGRFeature* Feature;
		L_TinTerrain->ResetReading();

		while((Feature = L_TinTerrain->GetNextFeature()) != NULL)
		{
			if(Feature->GetGeometryRef()->Intersects(PolyBBox))
				TinTerrain->addGeometry(Feature->GetGeometryRef());
		}

		L_TinTerrainEnvelope->ResetReading();

		while((Feature = L_TinTerrainEnvelope->GetNextFeature()) != NULL)
		{
			if(Feature->GetGeometryRef()->Intersects(PolyBBox))
				TinTerrainEnvelope->addGeometry(Feature->GetGeometryRef()->Intersection(PolyBBox));
		}

		std::cout << "TinTerrain et TinTerrainEnvelope charges." << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudGround (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoGround (new pcl::PointCloud<pcl::PointXYZ>);

		int cpt = 0;

		for(pcl::PointXYZ P : cloud->points)
		{
			std::cout << "Points traites : " << cpt << " / " <<  cloud->points.size() << std::endl;
			++cpt;
			OGRPoint* Point = new OGRPoint(P.x, P.y, P.z);
			if(!Point->Intersects(TinTerrainEnvelope))
				continue;
			for(int i = 0; i < TinTerrain->getNumGeometries(); ++i)
			{
				OGRPolygon* Poly = (OGRPolygon*)TinTerrain->getGeometryRef(i);
				if(!Point->Intersects(Poly))
					continue;

				OGRPoint* PointPoly = ProjectPointOnPolygon3D(Point, Poly);
				if(abs(P.z - PointPoly->getZ()) < 2)
					cloudGround->points.push_back(P);
				else
					cloudNoGround->points.push_back(P);
			}
		}

		cloudGround->width = (int)cloudGround->points.size();
		cloudGround->height = 1;
		cloudGround->is_dense = false;
		cloudNoGround->width = (int)cloudNoGround->points.size();
		cloudNoGround->height = 1;
		cloudNoGround->is_dense = false;

		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("1845_5181_Ground.pcd", *cloudGround, false);
		writer.write<pcl::PointXYZ> ("1845_5181_NoGround.pcd", *cloudNoGround, false);

		std::cout << "LiDAR separe en Ground et NoGround." << std::endl;
	}

	void Algo::CompareBuildings()
	{
		//Ouverture de la Bounding Box de la zone

		OGRDataSource* DS_BBox = OGRSFDriverRegistrar::Open("BoundingBox_1845_5181.shp");
		OGRLayer* L_BBox = DS_BBox->GetLayer(0);
		OGRFeature* Feature;
		L_BBox->ResetReading();
		OGRPolygon* BBox;
		while((Feature = L_BBox->GetNextFeature()) != NULL)
			BBox = (OGRPolygon*)Feature->GetGeometryRef()->clone();
		delete DS_BBox;

		//Ouverture du fichier LiDAR décomposé en ensemble de points voisins pouvant représenter des bâtiments

		pcl::PCDReader reader;
		pcl::PCDWriter writer;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		reader.read<pcl::PointXYZRGB> ("C:/Users/FredLiris/Downloads/1845_5181_NoGround_Trie_1_200.pcd", *cloud);

		std::vector<OGRMultiPoint*> ListCloud;
		OGRMultiPoint* PointsCloud = new OGRMultiPoint;
		double old_r = -1, old_g = -1, old_b = -1;

		OGRGeometryCollection* CloudPolygons = new OGRMultiPolygon;
		OGRGeometryCollection* CloudPolygons2 = new OGRMultiPolygon;

		int cpt = 0;

		for(pcl::PointXYZRGB Point : cloud->points)
		{
			if(Point.r != old_r || Point.g != old_g || Point.b != old_b)
			{
				std::cout << "Avancement : " << cpt << " / " << cloud->points.size() << std::endl;
				if(PointsCloud->getNumGeometries() > 0)
				{
					OGRGeometry* Test1 = PointsCloud->Buffer(2);
					OGRGeometry* Test2 = Test1->Buffer(-2);
					std::cout << Test2->getGeometryName() << std::endl; 
					if(Test2->getGeometryType() == wkbMultiPolygon || Test2->getGeometryType() == wkbMultiPolygon25D)//MultiPolygon à traiter : l'érosion peut couper en deux des polygones
					{
						OGRMultiPolygon* MP = (OGRMultiPolygon*)Test2;
						for(int p = 0; p < MP->getNumGeometries(); ++p)
							CloudPolygons->addGeometry(MP->getGeometryRef(p));
					}
					else
						CloudPolygons->addGeometry(Test2);

					CloudPolygons2->addGeometry(Test1);

					delete Test1;
					delete Test2;

					ListCloud.push_back((OGRMultiPoint*)PointsCloud->clone());
					delete PointsCloud;
					PointsCloud = new OGRMultiPoint;
				}
				old_r = Point.r;
				old_g = Point.g;
				old_b = Point.b;
			}
			PointsCloud->addGeometry(new OGRPoint(Point.x, Point.y, Point.z));
			++cpt;
		}

		SaveGeometrytoShape("CloudPolygons.shp", CloudPolygons);
		SaveGeometrytoShape("CloudPolygons2.shp", CloudPolygons2);

		OGRDataSource* DS = OGRSFDriverRegistrar::Open("CloudPolygons.shp");
		OGRLayer* Layer = DS->GetLayer(0);
		Layer->ResetReading();
		while((Feature = Layer->GetNextFeature()) != NULL)
			CloudPolygons->addGeometry(Feature->GetGeometryRef());

		//Ouverture des emprises au sol représentant les bâtiments connus

		OGRDataSource* DS_Batis = OGRSFDriverRegistrar::Open("BatisNew.shp");
		OGRLayer* L_Batis = DS_Batis->GetLayer(0);
		L_Batis->ResetReading();
		OGRMultiPolygon* ListBatis = new OGRMultiPolygon;

		while((Feature = L_Batis->GetNextFeature()) != NULL)
		{
			OGRPolygon* Bati = (OGRPolygon*)Feature->GetGeometryRef();

			if(!Bati->Intersects(BBox))
				continue;
			ListBatis->addGeometry(Bati);
		}	
		delete DS_Batis;

		//Comparaison des deux jeux de données

		OGRGeometryCollection* CloudPolygons3 = new OGRMultiPolygon;

		for(int i = 0; i < CloudPolygons->getNumGeometries(); ++i)
		{
			bool exist = false;
			for(int j = 0; j < ListBatis->getNumGeometries(); ++j)
			{
				if(CloudPolygons->getGeometryRef(i)->Intersects(ListBatis->getGeometryRef(j)))
				{
					exist = true;
					break;
				}
			}
			if(exist)
				CloudPolygons3->addGeometry(CloudPolygons->getGeometryRef(i));
		}

		SaveGeometrytoShape("CloudPolygons3.shp", CloudPolygons3);
	}

	void Algo::ConstructRoofs()
	{
		pcl::PCDReader reader;
		pcl::PCDWriter writer;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		reader.read<pcl::PointXYZRGB> ("C:/Users/FredLiris/Downloads/PCL/1845_5181_NoGround_Trie_1_200.pcd", *cloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr currcloud (new pcl::PointCloud<pcl::PointXYZ>);

		double old_r = -1, old_g = -1, old_b = -1;

		int cpt = 0;

		for(pcl::PointXYZRGB Point : cloud->points)
		{
			if(Point.r != old_r || Point.g != old_g || Point.b != old_b)
			{
				std::cout << "Avancement : " << cpt << " / " << cloud->points.size() << std::endl;
				if(currcloud->points.size() > 0)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud2(new pcl::PointCloud<pcl::PointXYZ>(*currcloud));
					Cloud2->width = (int)currcloud->points.size();
					Cloud2->height = 1;
					Cloud2->is_dense = false;
					writer.write<pcl::PointXYZ> ("CurrCloud.pcd", *Cloud2, false);
					//std::cout << "CurrCloud.pcd cree." << std::endl;

					SegmentationByNormal(currcloud);
					//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Planes = ExtractPlansFromCloud(currcloud);
					currcloud->clear();
					std::cout << "Continue ? " << std::endl;
					int a;
					std::cin >> a;
				}
				old_r = Point.r;
				old_g = Point.g;
				old_b = Point.b;
			}
			//std::cout << "Test1" << std::endl;
			currcloud->points.push_back(pcl::PointXYZ(Point.x, Point.y, Point.z));
			//std::cout << "Test2" << std::endl;
			++cpt;
		}
	}*/
