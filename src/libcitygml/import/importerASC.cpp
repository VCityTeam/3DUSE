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
CityModel* ImporterASC::reliefToCityGML(MNT* mnt)
{
	CityModel* model = new CityModel();
	CityObject* reliefFeature = new ReliefFeature("");
	CityObject* reliefTIN = new TINRelief("");

	reliefTIN->addGeometry(generateTriangles(mnt));

	model->addCityObject(reliefTIN);
	reliefFeature->getChildren().push_back(reliefTIN);
	model->addCityObject(reliefFeature);
	model->addCityObjectAsRoot(reliefFeature);
	model->computeEnvelope();
	std::cout<<"Conversion OK    "<<std::endl;
	return model;
}
/////////////////////////////////////////////////////////////////////////////////////////
CityModel* ImporterASC::waterToCityGML(MNT* mnt)
{
	CityModel* model = new CityModel();
	CityObject* waterbody = new WaterBody("");
	CityObject* watersfc = new WaterSurface("");

	watersfc->addGeometry(generateTriangles(mnt));

	waterbody->getChildren().push_back(watersfc);
	watersfc->_parent = waterbody;
	model->addCityObject(waterbody);
	model->addCityObjectAsRoot(waterbody);
	model->computeEnvelope();	
	std::cout<<"Conversion OK    "<<std::endl;
	return model;
}
/////////////////////////////////////////////////////////////////////////////////////////
Geometry* ImporterASC::generateTriangles(MNT* mnt)
{
	Geometry* geom = new Geometry("", GT_Unknown,3);
	for (int y=0; y<mnt->get_dim_y()-1;y++)
	{
		for (int x=0; x<mnt->get_dim_x()-1;x++)
		{
			bool emptyCell = true;
			TVec3d v1, v2, v3, v4;
			v1[0] = (mnt->get_x_noeud_NO())+x*(mnt->get_pas_x());
			v1[1] = (mnt->get_y_noeud_NO())+(-y)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
			v1[2] = mnt->get_altitude(x,y);
			v2[0] = (mnt->get_x_noeud_NO())+x*(mnt->get_pas_x());
			v2[1] = (mnt->get_y_noeud_NO())+(-y-1)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
			v2[2] = mnt->get_altitude(x,y+1);
			v3[0] = (mnt->get_x_noeud_NO())+(x+1)*(mnt->get_pas_x());
			v3[1] = (mnt->get_y_noeud_NO())+(-y)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
			v3[2] = mnt->get_altitude(x+1,y);
			v4[0] = (mnt->get_x_noeud_NO())+(x+1)*(mnt->get_pas_x());
			v4[1] = (mnt->get_y_noeud_NO())+(-y-1)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
			v4[2] = mnt->get_altitude(x+1,y+1);
			
			if(v1[2]!=(mnt->get_nodata()) && v2[2]!=(mnt->get_nodata()) && v3[2]!=(mnt->get_nodata()) )
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
			if(v2[2]!=(mnt->get_nodata()) && v3[2]!=(mnt->get_nodata()) && v4[2]!=(mnt->get_nodata()) )
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
			//	if(v1[2]!=(mnt->get_nodata()) && v2[2]!=(mnt->get_nodata()) && v4[2]!=(mnt->get_nodata()) )
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
			//	if(v1[2]!=(mnt->get_nodata()) && v3[2]!=(mnt->get_nodata()) && v4[2]!=(mnt->get_nodata()) )
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
		printf( "Conversion (%d%%)\r", (int)(y*100.0/mnt->get_dim_y()) );
		fflush(stdout);
	}
	return geom;
}
/////////////////////////////////////////////////////////////////////////////////////////
void ImporterASC::cutASC(MNT* mnt, std::string path, std::string filename, int tileSize)
{

	int x = 0;
	int y = mnt->get_dim_y()-1;

	while ( y>0 )
	{

		// get bounds of the tile we're in
		float realX = (mnt->get_x_noeud_NO())+x*(mnt->get_pas_x());
		float realY = (mnt->get_y_noeud_NO())+(-y)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
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
			if (iX+1>=mnt->get_dim_x()) break;
			nC++;
			realX = (mnt->get_x_noeud_NO())+(++iX)*(mnt->get_pas_x());
		}
		int nR = 1;
		int iY = y;
		while (tileYmin <= realY && realY < tileYmax)
		{
			if (iY<=0) break;
			nR++;
			realY = (mnt->get_y_noeud_NO())+(-(--iY))*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
		}

		std::string fname = path+"/"+filename+"T"+std::to_string(dvX)+"-"+std::to_string(dvY)+".asc";
		std::ofstream out;
		out.open(fname);

		//write out file header
		out<<"ncols         "<<nC<<std::endl;
		out<<"nrows         "<<nR<<std::endl;
		out<<"xllcorner     "<<std::fixed<<cornerX<<std::endl;
		out<<"yllcorner     "<<std::fixed<<cornerY<<std::endl;
		out<<"cellsize      "<<mnt->get_pas_x()<<std::endl;
		out<<"nodata_value  "<<mnt->get_nodata()<<std::endl;
		//parse data for this tile
		for (iY = nR-1; iY >= 0; iY--)
		{
			for (iX = 0; iX < nC; iX++)
			{
				//access & write data at x+iX, y-iY
				out<<mnt->get_altitude(x+iX,y-iY)<<" ";
			}
			out<<std::endl;
		}
		//tile is finished, set xy for next tile
		if ((x+nC)<mnt->get_dim_x())
		{
			x = x+nC-1; //same row, next column
		}
		else
		{
			x = 0; //next row, first column
			y = y-nR+1;
		}
		printf( "Conversion (%d%%)\r", (int)((mnt->get_dim_y()-y)*100.0/mnt->get_dim_y()) );
		fflush(stdout);
		out.close();
	}
	std::cout<<"Done!            "<<std::endl;

}
/////////////////////////////////////////////////////////////////////////////////////////
}//namespace citygml