#include "importerASC.hpp"

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
	CityObject* relief = new TINRelief("");

	relief->addGeometry(generateTriangles(mnt));

	model->addCityObject(relief);
	model->addCityObjectAsRoot(relief);
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
	const int incr = 1;
	Geometry* geom = new Geometry("", GT_Unknown,3);
	for (int y=0; y<mnt->get_dim_y()-incr;y+=incr)
	{
		for (int x=0; x<mnt->get_dim_x()-incr;x+=incr)
		{
			bool emptyCell = true;
			TVec3d v1, v2, v3, v4;
			v1[0] = (mnt->get_x_noeud_NO())+x*(mnt->get_pas_x());
			v1[1] = (mnt->get_y_noeud_NO())+(-y)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
			v1[2] = mnt->get_altitude(x,y);
			v2[0] = (mnt->get_x_noeud_NO())+x*(mnt->get_pas_x());
			v2[1] = (mnt->get_y_noeud_NO())+(-y-incr)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
			v2[2] = mnt->get_altitude(x,y+incr);
			v3[0] = (mnt->get_x_noeud_NO())+(x+incr)*(mnt->get_pas_x());
			v3[1] = (mnt->get_y_noeud_NO())+(-y)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
			v3[2] = mnt->get_altitude(x+incr,y);
			v4[0] = (mnt->get_x_noeud_NO())+(x+incr)*(mnt->get_pas_x());
			v4[1] = (mnt->get_y_noeud_NO())+(-y-incr)*(mnt->get_pas_y())+(mnt->get_dim_y()*mnt->get_pas_y());
			v4[2] = mnt->get_altitude(x+incr,y+incr);
			
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
}//namespace citygml