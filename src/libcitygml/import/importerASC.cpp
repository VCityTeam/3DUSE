#include "importerASC.hpp"
#include "src/processes/ExportToShape.hpp"

#include <osgDB/fstream>

namespace citygml
{

ImporterASC::ImporterASC(void)
{
}


ImporterASC::~ImporterASC(void)
{	
	if( altitudes ) delete[] altitudes;
}
/////////////////////////////////////////////////////////////////////////////////////////
bool ImporterASC::charge(const char* nom_fichier, const char* type_fichier)
{
	FILE	*fp;
	char	chaine[500];

	NODATA_value = -9999;

	bool mnt_charge = false;

	fp = fopen( nom_fichier, "rt" );
	if( !fp )
		return false;

	if( strcmp( type_fichier, "MNT" ) == 0 )
	{
		int r;

		r = fscanf( fp, "%s", chaine );				// "MNT"
		if( strcmp( chaine, "MNT" ) != 0 )
			return false;

		r = fscanf( fp, "%s", chaine );				// Numéro de version
		r = fscanf( fp, "%s", nom_chantier );			// Nom du chantier
		r = fscanf( fp, "%s", unites_xy );			// Unité des xy
		r = fscanf( fp, "%f", &precision_xy );			// Précision de l'unité
		r = fscanf( fp, "%f", &x_noeud_SO );			// x du noeud Nord-Ouest
		r = fscanf( fp, "%f", &y_noeud_SO );			// y du noeud Nord-Ouest
		r = fscanf( fp, "%f", &pas_x );				// pas en x
		r = fscanf( fp, "%f", &pas_y );				// pas en y
		r = fscanf( fp, "%d", &dim_y );				// nombre de lignes
		r = fscanf( fp, "%d", &dim_x );				// nombre de colonnes
		r = fscanf( fp, "%s", unites_z );			// Unité des z
		r = fscanf( fp, "%f", &precision_z );			// Précision de l'unité

		y_noeud_SO -= 3000000;
	}
	else if( strcmp( type_fichier, "ASC" ) == 0 )
	{
		int r;

		strcpy( nom_chantier, "empty" );

		r = fscanf( fp, "%s", chaine );
		if( strcmp( chaine, "ncols" ) == 0 )
		{
			r = fscanf( fp, "%d", &dim_x );				// nombre de colonnes
			printf("ncols: %d\n", dim_x);
		}
		r = fscanf( fp, "%s", chaine );
		if( strcmp( chaine, "nrows" ) == 0 )
		{
			r = fscanf( fp, "%d", &dim_y );				// nombre de lignes
			printf("nrows: %d\n", dim_y);
		}
		r = fscanf( fp, "%s", chaine );
		if( strcmp( chaine, "xllcorner" ) == 0 )
		{
			r = fscanf( fp, "%f", &x_noeud_SO );			// x du noeud Nord-Ouest
			printf("xllcorner: %f\n", x_noeud_SO);
		}
		r = fscanf( fp, "%s", chaine );
		if( strcmp( chaine, "yllcorner" ) == 0 )
		{
			r = fscanf( fp, "%f", &y_noeud_SO );			// y du noeud Nord-Ouest
			printf("yllcorner: %f\n", y_noeud_SO);
		}
		r = fscanf( fp, "%s", chaine );
		if( strcmp( chaine, "cellsize" ) == 0 )
		{
			r = fscanf( fp, "%f", &pas_x );				// pas en x
			printf("cellsize: %f\n", pas_x);
			pas_y = pas_x;						// pas en y
		}
		r = fscanf( fp, "%s", chaine );
		if( strcmp( chaine, "NODATA_value" ) == 0 || strcmp( chaine, "nodata_value" ) == 0 )
		{
			r = fscanf( fp, "%f", &NODATA_value );			// NODATA_value
			printf("NODATA_value: %f\n", NODATA_value);
		}
		else
		{
			fclose(fp);
			return false;
		}
	}
	else
	{
		fclose(fp);
		return false;
	}

	printf( "origine = (%f, %f)\n", x_noeud_SO, y_noeud_SO );

	altitudes = new float[dim_x*dim_y];

	// Lecture des altitudes
	int offset = 0;
	int r;
	for( int y=0; y<dim_y; y++ )
	{
		for( int x=0; x<dim_x; x++ )
		{
			float a = NODATA_value;
			r = fscanf( fp, "%f", &a );
			altitudes[offset] = a;
			//printf("%d \n", altitudes[offset]);
			offset++;
		}
		printf( "Chargement (%d%%)\r", (int)(y*100.0/dim_y) );
		fflush(stdout);
	}
	printf("Chargement OK    \n");

	fclose(fp);

	mnt_charge = true;
	return mnt_charge;
}
/////////////////////////////////////////////////////////////////////////////////////////
CityModel* ImporterASC::reliefToCityGML()
{
	CityModel* model = new CityModel();
	CityObject* reliefFeature = new ReliefFeature("");
	CityObject* reliefTIN = new TINRelief("");

	reliefTIN->addGeometry(generateTriangles());

	model->addCityObject(reliefTIN);
	reliefFeature->getChildren().push_back(reliefTIN);
	model->addCityObject(reliefFeature);
	model->addCityObjectAsRoot(reliefFeature);
	model->computeEnvelope();
	std::cout<<"Conversion OK    "<<std::endl;
	return model;
}
/////////////////////////////////////////////////////////////////////////////////////////
CityModel* ImporterASC::waterToCityGML()
{
	CityModel* model = new CityModel();
	CityObject* waterbody = new WaterBody("");
	CityObject* watersfc = new WaterSurface("");

	watersfc->addGeometry(generateTriangles());

	waterbody->getChildren().push_back(watersfc);
	watersfc->_parent = waterbody;
	model->addCityObject(waterbody);
	model->addCityObjectAsRoot(waterbody);
	model->computeEnvelope();	
	std::cout<<"Conversion OK    "<<std::endl;
	return model;
}
/////////////////////////////////////////////////////////////////////////////////////////
Geometry* ImporterASC::generateTriangles()
{
	int incrx = 1; // for manually changing step
	int incry = 1;
	Geometry* geom = new Geometry("", GT_Unknown,3);
	for (int y=0; y<dim_y-incry;y+=incry)
	{
		//incry=25-incry; //for alternative step
		//incrx=13;
		for (int x=0; x<dim_x-incrx;x+=incrx)
		{
			//incrx = 25-incrx; //for alternative step
			bool emptyCell = true;
			TVec3d v1, v2, v3, v4;
			v1[0] = (x_noeud_SO)+x*(pas_x);
			v1[1] = (y_noeud_SO)+(-y)*(pas_y)+(dim_y*pas_y);
			v1[2] = get_altitude(x,y);
			v2[0] = (x_noeud_SO)+x*(pas_x);
			v2[1] = (y_noeud_SO)+(-y-incry)*(pas_y)+(dim_y*pas_y);
			v2[2] = get_altitude(x,y+incry);
			v3[0] = (x_noeud_SO)+(x+incrx)*(pas_x);
			v3[1] = (y_noeud_SO)+(-y)*(pas_y)+(dim_y*pas_y);
			v3[2] = get_altitude(x+incrx,y);
			v4[0] = (x_noeud_SO)+(x+incrx)*(pas_x);
			v4[1] = (y_noeud_SO)+(-y-incry)*(pas_y)+(dim_y*pas_y);
			v4[2] = get_altitude(x+incrx,y+incry);

			if(v1[2]!=(NODATA_value) && v2[2]!=(NODATA_value) && v3[2]!=(NODATA_value) )
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
			if(v2[2]!=(NODATA_value) && v3[2]!=(NODATA_value) && v4[2]!=(NODATA_value) )
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
		printf( "Conversion (%d%%)\r", (int)(y*100.0/dim_y) );
		fflush(stdout);
	}
	return geom;
}
/////////////////////////////////////////////////////////////////////////////////////////
void ImporterASC::cutASC(std::string path, std::string filename, int tileSize)
{

	int x = 0;
	int y = dim_y-1;

	while ( y>0 )
	{

		// get bounds of the tile we're in
		float realX = (x_noeud_SO)+x*(pas_x);
		float realY = (y_noeud_SO)+(-y)*(pas_y)+(dim_y*pas_y);
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
			if (iX+1>=dim_x) break;
			nC++;
			realX = (x_noeud_SO)+(++iX)*(pas_x);
		}
		int nR = 1;
		int iY = y;
		while (tileYmin <= realY && realY < tileYmax)
		{
			if (iY<=0) break;
			nR++;
			realY = (y_noeud_SO)+(-(--iY))*(pas_y)+(dim_y*pas_y);
		}

		std::string fname = path+"/"+filename+"T"+std::to_string(dvX)+"-"+std::to_string(dvY)+".asc";
		std::ofstream out;
		out.open(fname);

		//write out file header
		out<<"ncols         "<<nC<<std::endl;
		out<<"nrows         "<<nR<<std::endl;
		out<<"xllcorner     "<<std::fixed<<cornerX<<std::endl;
		out<<"yllcorner     "<<std::fixed<<cornerY<<std::endl;
		out<<"cellsize      "<<pas_x<<std::endl;
		out<<"nodata_value  "<<NODATA_value<<std::endl;
		//parse data for this tile
		for (iY = nR-1; iY >= 0; iY--)
		{
			for (iX = 0; iX < nC; iX++)
			{
				//access & write data at x+iX, y-iY
				out<<get_altitude(x+iX,y-iY)<<" ";
			}
			out<<std::endl;
		}
		//tile is finished, set xy for next tile
		if ((x+nC)<dim_x)
		{
			x = x+nC-1; //same row, next column
		}
		else
		{
			x = 0; //next row, first column
			y = y-nR+1;
		}
		printf( "Conversion (%d%%)\r", (int)((dim_y-y)*100.0/dim_y) );
		fflush(stdout);
		out.close();
	}
	std::cout<<"Done!            "<<std::endl;

}
/////////////////////////////////////////////////////////////////////////////////////////
CityModel* ImporterASC::waterToCityGMLPolygons()
{
	CityModel* model = new CityModel();
	CityObject* waterbody = new WaterBody("");
	//CityObject* watersfc = new WaterSurface("");
	//Geometry* geom = new Geometry("", GT_Unknown,3);

	treated = new bool[dim_x*dim_y];
	for (int i =0; i<(dim_x*dim_y);i++) treated[i]=false;
	for (int y=0; y<dim_y;y++)
	{
		for (int x=0; x<dim_x;x++)
		{
			if (!treated[x+y*dim_x] && get_altitude(x,y)!=NODATA_value)
			{
				CityObject* watersfc = new WaterSurface("");
				Geometry* geom = new Geometry("", GT_Unknown,3);

				geom_list.clear();
				
				OGRMultiPolygon* pOgrMerged = new OGRMultiPolygon;
				std::deque<std::pair<int, int>> pointsList;
				pointsList.push_back(std::make_pair(x,y));
				treated[x+y*dim_x]=true;
				while (!pointsList.empty())
				{
					pOgrMerged->addGeometryDirectly(createPoly(pointsList.front().first,pointsList.front().second));
					pointsList.pop_front();
					propagateCategory(&pointsList);
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
		std::cout<<"Traitement ("<<(int)(y*100.0/dim_y)<<"%)\r";
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
void ImporterASC::propagateCategory(std::deque<std::pair<int,int>>* pointsList)
{
	int x = pointsList->front().first;
	int y = pointsList->front().second;
	//std::cout<<x<<";"<<y<<" / "<<dim_x<<";"<<dim_y<<std::endl;

	// If some neighbours have same altitude but no category
	float alt = get_altitude(x,y);
	if (y>0 && !treated[x+(y-1)*dim_x] && abs(get_altitude(x,y-1)-alt)<0.001)// up
	{
		pointsList->push_back(std::make_pair(x,y-1));
		treated[x+(y-1)*dim_x]=true;
	} 
	if (x>0 && !treated[x-1+y*dim_x] && abs(get_altitude(x-1,y)-alt)<0.001)// left
	{
		pointsList->push_back(std::make_pair(x-1,y));
		treated[x-1+y*dim_x]=true;
	} 
	if ((y+1)<dim_y && !treated[x+(y+1)*dim_x] && abs(get_altitude(x,y+1)-alt)<0.001)// down
	{
		pointsList->push_back(std::make_pair(x,y+1));
		treated[x+(y+1)*dim_x]=true;
	} 
	if ((x+1)<dim_x && !treated[x+1+y*dim_x] && abs(get_altitude(x+1,y)-alt)<0.001)// right
	{
		pointsList->push_back(std::make_pair(x+1,y));
		treated[x+1+y*dim_x]=true;
	} 
}
/////////////////////////////////////////////////////////////////////////////////////////
OGRPolygon* ImporterASC::createPoly(int x, int y)
{
	OGRLinearRing* ring = new OGRLinearRing;

	float xmin = x_noeud_SO+x*pas_x;
	float ymax = y_noeud_SO+dim_y*pas_y-y*pas_y;
	float xmax = x_noeud_SO+(x+1)*pas_x;
	float ymin = y_noeud_SO+dim_y*pas_y-(y+1)*pas_y;

	ring->addPoint(xmin, ymin, floor(get_altitude(x,y)*1000+0.5)/1000);
	ring->addPoint(xmin, ymax, floor(get_altitude(x,y)*1000+0.5)/1000);
	ring->addPoint(xmax, ymax, floor(get_altitude(x,y)*1000+0.5)/1000);
	ring->addPoint(xmax, ymin, floor(get_altitude(x,y)*1000+0.5)/1000);
	ring->addPoint(xmin, ymin, floor(get_altitude(x,y)*1000+0.5)/1000);

	OGRPolygon* poly = new OGRPolygon();
	poly->addRingDirectly(ring);
	return poly;
}
/////////////////////////////////////////////////////////////////////////////////////////
}//namespace citygml