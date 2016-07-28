#include "VegetTool.hpp"

#include <qfiledialog.h>
#include "src/utils/OGRGDAL_Utils/OGRGDALtoShpWriter.hpp"
#include "lasreader.hpp"

#include "citygml.hpp"
#include "export/exportCityGML.hpp"

std::string ProcessLasShpVeget(std::string dirTile)
{
	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load Shp file");

	QFileInfo file(filepath);

	QString ext = file.suffix().toLower();

	std::cout << "Shp : " << filepath.toStdString() << std::endl;

	std::string ouputfileName = "";

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
			std::map<OGRPolygon*, std::string> polyid;

			//We are going to read the shp and store all polygon to a vector
			//=======<ShpReading>========
			OGRLayer *poLayer;
			int nbLayers = poDS->GetLayerCount();
			if(nbLayers > 0)
			{
				poLayer = poDS->GetLayer(0);

				OGRFeature *poFeature;
				poLayer->ResetReading();

				unsigned int cpt = 0;

				while( (poFeature = poLayer->GetNextFeature()) != NULL )
				{
					std::string name="noid";
					if(poFeature->GetFieldIndex("gid") != -1)
						name = poFeature->GetFieldAsString("gid");

					OGRGeometry* poGeometry = poFeature->GetGeometryRef();

					if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
					{
						//Emprise au sol
						OGRPolygon* poPG = (OGRPolygon*) poGeometry;
						vegetPoly.push_back(poPG);
						polyid.insert(std::make_pair(poPG,name));
					}
				}
			}
			//=======<ShpReading>========

			std::cout << "Veget Poly Vector Done." << std::endl;
			std::cout << "Processing LAS." << std::endl;

			std::map<OGRPolygon*,std::vector<OGRPoint>> points;//Polygon with it points from lidar.

			//For every point
			while (lasreader->read_point())
			{
				OGRPoint* ptemp = new OGRPoint((lasreader->point).get_x(),(lasreader->point).get_y(),(lasreader->point).get_z());
				bool found = false;
				for(unsigned int i = 0; i < vegetPoly.size(); i++)//For every poly
				{
					if(vegetPoly[i]->Contains(ptemp))//Checking if the point is in the poly
					{
						points[vegetPoly[i]].push_back(*ptemp);
						found = true;
						break;
					}
				}
				if(!found)
					delete ptemp;
			}

			std::cout << "Done." << std::endl;
			std::cout << "Exporting Dat File." << std::endl;

			std::filebuf fb;
			ouputfileName = dirTile+"/SkylineOutput/"+fileBis.baseName().toStdString()+".dat";
			fb.open(ouputfileName,std::ios::out);

			std::ostream file(&fb);

			file << points.size() << "\n";

			for(auto it = points.begin(); it != points.end(); it++)
			{
				file << it->second.size() << "\n";
				file << polyid[it->first] << "\n";
				for(OGRPoint p : it->second)
				{
					file << std::fixed << p.getX() << "\n";
					file << std::fixed << p.getY() << "\n";
					file << std::fixed << p.getZ() << "\n";
				}
			}

			fb.close();

			std::cout << "Done Exporting Dat File." << std::endl;

		}

	}

	return ouputfileName;
}

unsigned int ClampIt(unsigned int x,unsigned int a,unsigned int b)
{
	return x < a ? a : (x > b ? b : x);
}

citygml::CityObject* VegToCityObject(std::pair<std::string,std::vector<TVec3d>> veg, unsigned int cpt)
{
	std::string name = veg.first;
	float cellSize = 2.0;//in meter
	unsigned int ii = 0;
	unsigned int jj = 0;

	//Get and store the bounding box of the cloud point
	double xmin = DBL_MAX;
	double ymin = DBL_MAX;
	double xmax = -DBL_MAX;
	double ymax = -DBL_MAX;

	for(TVec3d p : veg.second)
	{
		xmin = std::min(xmin,p.x);
		ymin = std::min(ymin,p.y);
		xmax = std::max(xmax,p.x);
		ymax = std::max(ymax,p.y);
	}

	//Float dimension of box
	float widthtemp = ceil(xmax - xmin);
	float heighttemp = ceil(ymax - ymin);

	//How many cell we have
	unsigned int cellwidthcount = static_cast<unsigned int>(widthtemp/cellSize);
	unsigned int cellheightcount = static_cast<unsigned int>(heighttemp/cellSize);

	std::vector<std::vector<double>> table(cellwidthcount,std::vector<double>(cellheightcount,0.0));//Where heights are stored
	std::vector<std::vector<unsigned int>> tableCount(cellwidthcount,std::vector<unsigned int>(cellheightcount,0));//Where how many heights we have stored in table

	if(cellwidthcount > 0.0 && cellheightcount > 0.0)
	{

		for(TVec3d p : veg.second)
		{
			unsigned int xt = ClampIt(static_cast<unsigned int>((p.x - xmin) / cellSize),0,cellwidthcount-1);
			unsigned int yt = ClampIt(static_cast<unsigned int>((p.y - ymin) / cellSize),0,cellheightcount-1);

			table[xt][yt] += p.z;
			tableCount[xt][yt]++;
		}

		for(unsigned int i = 0; i < cellwidthcount; i++)
		{
			for(unsigned int j = 0; j < cellheightcount; j++)
			{
				if(tableCount[i][j] > 0)
				{
					table[i][j] = table[i][j] / tableCount[i][j];
				}
			}
		}


		citygml::CityObject* BuildingCO = new citygml::SolitaryVegetationObject(name);

		citygml::Geometry* cityGeom = new citygml::Geometry(name+"_Geometry", citygml::GT_Unknown, 2);

		for(unsigned int i = 0; i < cellwidthcount - 1; i++)
		{
			for(unsigned int j = 0; j < cellheightcount - 1; j++)
			{
				double iReal = i*cellSize;
				double jReal = j*cellSize;
				double ipReal = (i+1)*cellSize;
				double jpReal = (j+1)*cellSize;

				if(table[i][j] > 0.0 && table[i+1][j] > 0.0 && table[i+1][j+1] > 0.0 && table[i][j+1] > 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(iReal+xmin,jReal+ymin,table[i][j]));
					ring->addVertex(TVec3d(ipReal+xmin,jReal+ymin,table[i+1][j]));
					ring->addVertex(TVec3d(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
					ring->addVertex(TVec3d(iReal+xmin,jpReal+ymin,table[i][j+1]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
				else if(table[i][j] > 0.0 && table[i+1][j] > 0.0 && table[i+1][j+1] > 0.0 && table[i][j+1] == 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(iReal+xmin,jReal+ymin,table[i][j]));
					ring->addVertex(TVec3d(ipReal+xmin,jReal+ymin,table[i+1][j]));
					ring->addVertex(TVec3d(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
				else if(table[i][j] > 0.0 && table[i+1][j] > 0.0 && table[i+1][j+1] == 0.0 && table[i][j+1] > 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(iReal+xmin,jReal+ymin,table[i][j]));
					ring->addVertex(TVec3d(ipReal+xmin,jReal+ymin,table[i+1][j]));
					ring->addVertex(TVec3d(iReal+xmin,jpReal+ymin,table[i][j+1]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
				else if(table[i][j] > 0.0 && table[i+1][j] == 0.0 && table[i+1][j+1] > 0.0 && table[i][j+1] > 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(iReal+xmin,jReal+ymin,table[i][j]));
					ring->addVertex(TVec3d(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
					ring->addVertex(TVec3d(iReal+xmin,jpReal+ymin,table[i][j+1]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
				else if(table[i][j] == 0.0 && table[i+1][j] > 0.0 && table[i+1][j+1] > 0.0 && table[i][j+1] > 0.0)
				{
					{citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
					ring->addVertex(TVec3d(ipReal+xmin,jReal+ymin,table[i+1][j]));
					ring->addVertex(TVec3d(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
					ring->addVertex(TVec3d(iReal+xmin,jpReal+ymin,table[i][j+1]));
					citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
					poly->addRing(ring);
					cityGeom->addPolygon(poly);}
				}
			}
		}
	
		BuildingCO->addGeometry(cityGeom);

		

		return BuildingCO;
	}
	else
		return nullptr;
	
}

void ProcessCL(std::string filename)
{
	QFileInfo fileInfo(filename.c_str());
	std::string ouputname = fileInfo.baseName().toStdString();

	//Load the cloud point from a file
	std::string path = filename;

	char line[256];

	std::ifstream ifs (path, std::ifstream::in);

	ifs.getline(line,256);

	unsigned int count = atoi(line);

	std::vector<std::pair<std::string,std::vector<TVec3d>>> vegets;

	for(unsigned int i = 0; i < count; i++)
	{
		std::vector<TVec3d> veg;
		ifs.getline(line,256);
		unsigned int countBis = atoi(line);
		
		ifs.getline(line,256);
		std::string name = std::string(line);

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

		vegets.push_back(std::make_pair(name,veg));
	}

	ifs.close();

	ProcessCL(vegets,ouputname);
}

void ProcessCL(std::vector<std::pair<std::string,std::vector<TVec3d>>> vegets, std::string output)
{
	std::cout << "Trying to process" << std::endl;

	citygml::CityModel* ModelOut = new citygml::CityModel;

	std::cout << "To process : " << vegets.size() << std::endl;

	for(unsigned int i = 0; i < vegets.size(); i++)
	{
		std::cout << "Processing : " << i << std::endl;
		citygml::CityObject* BuildingCO = VegToCityObject(vegets[i],i);
		if(BuildingCO != nullptr)
		{
			ModelOut->addCityObject(BuildingCO);
			ModelOut->addCityObjectAsRoot(BuildingCO);
		}
	}

	citygml::ExporterCityGML exporter("./SkylineOutput/"+output+".gml");
	exporter.exportCityModel(*ModelOut);

	std::cout << "It is done !" << std::endl;
}