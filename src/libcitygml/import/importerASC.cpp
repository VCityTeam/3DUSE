#include "importerASC.hpp"

#include <osgDB/fstream>

namespace citygml
{

ImporterASC::ImporterASC(void)
{
}

ImporterASC::~ImporterASC(void)
{
}
/////////////////////////////////////////////////////////////////////////////////////////
CityModel* ImporterASC::reliefToCityGML(MNT* asc)
{
	CityModel* model = new CityModel();
	CityObject* reliefFeature = new ReliefFeature("");
	CityObject* reliefTIN = new TINRelief("");

	reliefTIN->addGeometry(generateTriangles(asc));

	model->addCityObject(reliefTIN);
	reliefFeature->getChildren().push_back(reliefTIN);
	model->addCityObject(reliefFeature);
	model->addCityObjectAsRoot(reliefFeature);
	model->computeEnvelope();
	std::cout<<"Conversion OK    "<<std::endl;
	return model;
}
/////////////////////////////////////////////////////////////////////////////////////////
CityModel* ImporterASC::waterToCityGML(MNT* asc)
{
	CityModel* model = new CityModel();
	CityObject* waterbody = new WaterBody("");
	CityObject* watersfc = new WaterSurface("");

	watersfc->addGeometry(generateTriangles(asc));

	waterbody->getChildren().push_back(watersfc);
	watersfc->_parent = waterbody;
	model->addCityObject(waterbody);
	model->addCityObjectAsRoot(waterbody);
	model->computeEnvelope();	
	std::cout<<"Conversion OK    "<<std::endl;
	return model;
}
/////////////////////////////////////////////////////////////////////////////////////////
Geometry* ImporterASC::generateTriangles(MNT* asc)
{
	int incrx = 1; // for manually changing step
	int incry = 1;
	Geometry* geom = new Geometry("", GT_Unknown,3);
	for (int y=0; y<asc->get_dim_y()-incry;y+=incry)
	{
		//incry=25-incry; //for alternative step
		//incrx=13;
		for (int x=0; x<asc->get_dim_x()-incrx;x+=incrx)
		{
			//incrx = 25-incrx; //for alternative step
			bool emptyCell = true;
			float xmin = (asc->get_x_noeud_NO())+(x+0.5)*(asc->get_pas_x());
			float xmax = (asc->get_x_noeud_NO())+(x+incrx+0.5)*(asc->get_pas_x());
			float ymax = (asc->get_y_noeud_NO())+(-y-0.5)*(asc->get_pas_y())+(asc->get_dim_y()*asc->get_pas_y());
			float ymin = (asc->get_y_noeud_NO())+(-y-incry-0.5)*(asc->get_pas_y())+(asc->get_dim_y()*asc->get_pas_y());
			TVec3d v1, v2, v3, v4;
			v1[0] = xmin;
			v1[1] = ymax;
			v1[2] = asc->get_altitude(x,y);
			v2[0] = xmin;
			v2[1] = ymin;
			v2[2] = asc->get_altitude(x,y+incry);
			v3[0] = xmax;
			v3[1] = ymax;
			v3[2] = asc->get_altitude(x+incrx,y);
			v4[0] = xmax;
			v4[1] = ymin;
			v4[2] = asc->get_altitude(x+incrx,y+incry);

			if(v1[2]!=(asc->get_nodata()) && v2[2]!=(asc->get_nodata()) && v3[2]!=(asc->get_nodata()) )
			{
				Polygon* t = new Polygon("");
				LinearRing* lr = new LinearRing("",true);
				//std::vector<TVec3d> vec = lr->getVertices();
				lr->getVertices().push_back( v1 );
				lr->getVertices().push_back( v2 );
				lr->getVertices().push_back( v3 );
				t->addRing( lr );
				geom->addPolygon( t );
				emptyCell = false;
			}
			if(v2[2]!=(asc->get_nodata()) && v3[2]!=(asc->get_nodata()) && v4[2]!=(asc->get_nodata()) )
			{
				Polygon* t = new Polygon("");
				LinearRing* lr = new LinearRing("",true);
				//std::vector<TVec3d> vec = lr->getVertices();
				lr->getVertices().push_back( v3 );
				lr->getVertices().push_back( v2 );
				lr->getVertices().push_back( v4 );
				t->addRing( lr );
				geom->addPolygon( t );
				emptyCell = false;
			}
			//if (emptyCell) //on regarde si on peut pas faire un triangle dans l'autre sens
			//{
			//	if(v1[2]!=(NODATA_value) && v2[2]!=(NODATA_value) && v4[2]!=(NODATA_value) )
			//	{
			//		Polygon* t = new Polygon("");
			//		LinearRing* lr = new LinearRing("",true);
			//		//std::vector<TVec3d> vec = lr->getVertices();
			//		lr->getVertices().push_back( v1 );
			//		lr->getVertices().push_back( v2 );
			//		lr->getVertices().push_back( v4 );
			//		t->addRing( lr );
			//		geom->addPolygon( t );
			//		emptyCell = false;
			//	}
			//	if(v1[2]!=(NODATA_value) && v3[2]!=(NODATA_value) && v4[2]!=(NODATA_value) )
			//	{
			//		Polygon* t = new Polygon("");
			//		LinearRing* lr = new LinearRing("",true);
			//		//std::vector<TVec3d> vec = lr->getVertices();
			//		lr->getVertices().push_back( v1 );
			//		lr->getVertices().push_back( v3 );
			//		lr->getVertices().push_back( v4 );
			//		t->addRing( lr );
			//		geom->addPolygon( t );
			//		emptyCell = false;
			//	}
			//}
		}
		printf( "Conversion (%d%%)\r", (int)(y*100.0/asc->get_dim_y()) );
		fflush(stdout);
	}
	return geom;
}
/////////////////////////////////////////////////////////////////////////////////////////
void ImporterASC::cutASC(MNT* asc, std::string path, std::string filename, int tileSize)
{

	int x = 0;
	int y = asc->get_dim_y()-1;

	while ( y>0 )
	{

		// get bounds of the tile we're in
		float realX = (asc->get_x_noeud_NO())+x*(asc->get_pas_x());
		float realY = (asc->get_y_noeud_NO())+(-y)*(asc->get_pas_y())+(asc->get_dim_y()*asc->get_pas_y());
		float cornerX = realX;
		float cornerY = realY;
		int dvX = realX/tileSize;
		int dvY = realY/tileSize;
		float tileXmin = dvX*tileSize;
		float tileXmax = (dvX+1)*tileSize;
		float tileYmin = dvY*tileSize;
		float tileYmax = (dvY+1)*tileSize;
		/*std::cout<<"x = "<<realX<<" | y = "<<realY<<std::endl;
		std::cout<<"Tile : X = "<<tileXmin<<" -> "<<tileXmax<<std::endl;
		std::cout<<"       Y = "<<tileYmin<<" -> "<<tileYmax<<std::endl<<std::endl;*/

		//get number of rows and columns in this tile
		int nC = 1;
		int iX = x;
		while (tileXmin <= realX && realX < tileXmax)
		{
			if (iX+1>=asc->get_dim_x()) break;
			nC++;
			realX = (asc->get_x_noeud_NO())+(++iX)*(asc->get_pas_x());
		}
		int nR = 1;
		int iY = y;
		while (tileYmin <= realY && realY < tileYmax)
		{
			if (iY<=0) break;
			nR++;
			realY = (asc->get_y_noeud_NO())+(-(--iY))*(asc->get_pas_y())+(asc->get_dim_y()*asc->get_pas_y());
		}

		std::string fname = path+"/"+filename+"T"+std::to_string(dvX)+"-"+std::to_string(dvY)+".asc";
		std::ofstream out;
		out.open(fname);

		//write out file header
		out<<"ncols         "<<nC<<std::endl;
		out<<"nrows         "<<nR<<std::endl;
		out<<"xllcorner     "<<std::fixed<<cornerX<<std::endl;
		out<<"yllcorner     "<<std::fixed<<cornerY<<std::endl;
		out<<"cellsize      "<<asc->get_pas_x()<<std::endl;
		out<<"nodata_value  "<<asc->get_nodata()<<std::endl;
		//parse data for this tile
		for (iY = nR-1; iY >= 0; iY--)
		{
			for (iX = 0; iX < nC; iX++)
			{
				//access & write data at x+iX, y-iY
				out<<asc->get_altitude(x+iX,y-iY)<<" ";
			}
			out<<std::endl;
		}
		//tile is finished, set xy for next tile
		if ((x+nC)<asc->get_dim_x())
		{
			x = x+nC-1; //same row, next column
		}
		else
		{
			x = 0; //next row, first column
			y = y-nR+1;
		}
		printf( "Conversion (%d%%)\r", (int)((asc->get_dim_y()-y)*100.0/asc->get_dim_y()) );
		fflush(stdout);
		out.close();
	}
	std::cout<<"Done!            "<<std::endl;

}
/////////////////////////////////////////////////////////////////////////////////////////
CityModel* ImporterASC::waterToCityGMLPolygons(MNT* asc)
{
	CityModel* model = new CityModel();
	CityObject* waterbody = new WaterBody("");
	//CityObject* watersfc = new WaterSurface("");
	//Geometry* geom = new Geometry("", GT_Unknown,3);

	const float zPrec = 0.1;

	treated = new bool[asc->get_dim_x()*asc->get_dim_y()];
	for (int i =0; i<(asc->get_dim_x()*asc->get_dim_y());i++) treated[i]=false;
	for (int y=0; y<asc->get_dim_y();y++)
	{
		for (int x=0; x<asc->get_dim_x();x++)
		{
			if (!treated[x+y*asc->get_dim_x()] && asc->get_altitude(x,y)!=asc->get_nodata())
			{
				CityObject* watersfc = new WaterSurface("");
				Geometry* geom = new Geometry("", GT_Unknown,3);

				geom_list.clear();
				
				OGRMultiPolygon* pOgrMerged = new OGRMultiPolygon;
				std::deque<std::pair<int, int>> pointsList;
				pointsList.push_back(std::make_pair(x,y));
				treated[x+y*asc->get_dim_x()]=true;
				float alt = asc->get_altitude(x,y);
				while (!pointsList.empty())
				{
					pOgrMerged->addGeometryDirectly(createPoly(asc,pointsList.front().first,pointsList.front().second,1/zPrec));
					propagateCategory(asc,&pointsList,alt,zPrec);
					pointsList.pop_front();
				}
				
				OGRGeometry* pTemp = pOgrMerged->UnionCascaded();

				OGRPolygon* poPG = (OGRPolygon*) pTemp;
				if(poPG == nullptr)
				{
					std::cout << " ERREUR pOgrMerged nest pas un polygone ! " << poPG->getGeometryName() << std::endl;
					int a;
					std::cin >> a;
				}
				//-------------------------------
				//get interior and exterior rings
				std::vector<TVec3d> extRing;
				OGRPoint p;
				TVec3d v;
				OGRLinearRing* poLR = poPG->getExteriorRing();
				for(int i=0; i<poLR->getNumPoints(); ++i)
				{
					poLR->getPoint(i, &p);
					v = TVec3d(p.getX(), p.getY(), p.getZ());
					extRing.push_back(v);
				}
				std::vector<std::vector<TVec3d>> intRing;

				for(unsigned int i = 0; i < (unsigned int)poPG->getNumInteriorRings();i++)
				{
					std::vector<TVec3d> intRingTemp;
					poLR = poPG->getInteriorRing(i);
					for(int i=0; i<poLR->getNumPoints(); ++i)
					{
						poLR->getPoint(i, &p);
						v = TVec3d(p.getX(), p.getY(), p.getZ());
						intRingTemp.push_back(v);
					}
					if(intRingTemp.size() == 0)
						continue;
					intRing.push_back(intRingTemp);

				}
				//build CityGML polygon
				citygml::Polygon* GMLpoly = new citygml::Polygon("");
				citygml::LinearRing* ring1 = new citygml::LinearRing("",true);
				for(unsigned int j = 0; j < extRing.size(); j++)
				{
					ring1->addVertex(extRing[j]);
				}
				GMLpoly->addRing(ring1);
				for(std::vector<TVec3d> vec : intRing)
				{
					citygml::LinearRing* ring2 = new citygml::LinearRing("",false);
					for(unsigned int j = 0; j < vec.size(); j++)
					{
						ring2->addVertex(vec[j]);
					}
					GMLpoly->addRing(ring2);
				}
				geom->addPolygon(GMLpoly);
				//---------------------------------------
				if (geom->size()!=0)
				{
					watersfc->addGeometry(geom);
					waterbody->getChildren().push_back(watersfc);
					watersfc->_parent = waterbody;
				}
				delete pOgrMerged;
				delete poPG;
			}
		}
		std::cout<<"Traitement ("<<(int)(y*100.0/asc->get_dim_y())<<"%)\r";
		fflush(stdout);
	}
	std::cout<<"Traitement OK!     "<<std::endl;
	delete treated;

	//end treatement and create CityModel
	//if (geom->size()!=0)
	//{
	//	watersfc->addGeometry(geom);
	//	waterbody->getChildren().push_back(watersfc);
	//	watersfc->_parent = waterbody;
		model->addCityObject(waterbody);
		model->addCityObjectAsRoot(waterbody);
	//}

	model->computeEnvelope();
	return model;
}
/////////////////////////////////////////////////////////////////////////////////////////
void ImporterASC::propagateCategory(MNT* asc, std::deque<std::pair<int,int>>* pointsList, float alt, float prec = 1)
{
	int x = pointsList->front().first;
	int y = pointsList->front().second;
	//std::cout<<x<<";"<<y<<" / "<<dim_x<<";"<<dim_y<<std::endl;

	// If some neighbours have same altitude but no category
	if (y>0 && !treated[x+(y-1)*asc->get_dim_x()] && abs(asc->get_altitude(x,y-1)-alt)<prec)// up
	{
		pointsList->push_back(std::make_pair(x,y-1));
		treated[x+(y-1)*asc->get_dim_x()]=true;
	} 
	if (x>0 && !treated[x-1+y*asc->get_dim_x()] && abs(asc->get_altitude(x-1,y)-alt)<prec)// left
	{
		pointsList->push_back(std::make_pair(x-1,y));
		treated[x-1+y*asc->get_dim_x()]=true;
	} 
	if ((y+1)<asc->get_dim_y() && !treated[x+(y+1)*asc->get_dim_x()] && abs(asc->get_altitude(x,y+1)-alt)<prec)// down
	{
		pointsList->push_back(std::make_pair(x,y+1));
		treated[x+(y+1)*asc->get_dim_x()]=true;
	} 
	if ((x+1)<asc->get_dim_x() && !treated[x+1+y*asc->get_dim_x()] && abs(asc->get_altitude(x+1,y)-alt)<prec)// right
	{
		pointsList->push_back(std::make_pair(x+1,y));
		treated[x+1+y*asc->get_dim_x()]=true;
	} 
}
/////////////////////////////////////////////////////////////////////////////////////////
OGRPolygon* ImporterASC::createPoly(MNT* asc, int x, int y, float prec = 1)
{
	OGRLinearRing* ring = new OGRLinearRing;

	float xmin = asc->get_x_noeud_NO() + x*asc->get_pas_x();
	float ymax = asc->get_y_noeud_NO() + asc->get_dim_y() * asc->get_pas_y() -y * asc->get_pas_y();
	float xmax = asc->get_x_noeud_NO() + (x+1)*asc->get_pas_x();
	float ymin = asc->get_y_noeud_NO() + asc->get_dim_y() * asc->get_pas_y() - (y+1)*asc->get_pas_y();

	ring->addPoint(xmin, ymin, floor(asc->get_altitude(x,y)*prec+0.5)/prec);
	ring->addPoint(xmin, ymax, floor(asc->get_altitude(x,y)*prec+0.5)/prec);
	ring->addPoint(xmax, ymax, floor(asc->get_altitude(x,y)*prec+0.5)/prec);
	ring->addPoint(xmax, ymin, floor(asc->get_altitude(x,y)*prec+0.5)/prec);
	ring->addPoint(xmin, ymin, floor(asc->get_altitude(x,y)*prec+0.5)/prec);

	OGRPolygon* poly = new OGRPolygon();
	poly->addRingDirectly(ring);
	return poly;
}
/////////////////////////////////////////////////////////////////////////////////////////
}//namespace citygml