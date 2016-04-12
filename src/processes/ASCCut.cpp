#include "ASCCut.hpp"
#include <iostream>
#include <osgDB/fstream>

void ASCCut(MNT* asc, int tileSizeX, int tileSizeY, std::string path, std::string filename)
{

	int x = 0;
	int y = asc->get_dim_y()-1;

	while ( y>0 )
	{

		// get bounds of the tile we're in
		float realX = (asc->get_x_noeud_NO())+(x+0.5)*(asc->get_pas_x());
		float realY = (asc->get_y_noeud_NO())+(asc->get_dim_y()-y-0.5)*(asc->get_pas_y());
		float cornerX = (asc->get_x_noeud_NO())+x*(asc->get_pas_x());
		float cornerY = (asc->get_y_noeud_NO())+(asc->get_dim_y()-y-1)*(asc->get_pas_y());
		int dvX = cornerX/tileSizeX;
		int dvY = cornerY/tileSizeY;
		float tileXmin = dvX*tileSizeX;
		float tileXmax = (dvX+1)*tileSizeY;
		float tileYmin = dvY*tileSizeX;
		float tileYmax = (dvY+1)*tileSizeY;

		//get number of rows and columns in this tile
		int nC = 1;
		int iX = x;
		while (tileXmin <= realX && realX < tileXmax)
		{
			if (iX+1>=asc->get_dim_x()) break;
			nC++;
			realX = (asc->get_x_noeud_NO())+(++iX+0.5)*(asc->get_pas_x());
		}
		int nR = 1;
		int iY = y;
		while (tileYmin <= realY && realY < tileYmax)
		{
			if (iY<=0) break;
			nR++;
			realY = (asc->get_y_noeud_NO())+(asc->get_dim_y()-(--iY)-0.5)*(asc->get_pas_y());
		}

		//QDir dir = QString::fromStdString(path);
		//QString tileName = QString::fromStdString("T"+std::to_string(dvX)+"-"+std::to_string(dvY));
		//if (!dir.exists(tileName))
		//dir.mkdir(tileName);
		//std::string fname = path+"/"+tileName.toStdString()+"/"+tileName.toStdString()+"_"+filename+".asc";
		std::string fname = path+"/"+std::to_string(dvX)+"-"+std::to_string(dvY)+"_"+filename+".asc";
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
		std::cout<<"Tiling ("<<(int)((asc->get_dim_y()-y)*100.0/asc->get_dim_y())<<"%)\r";
		fflush(stdout);
		out.close();
	}
	std::cout<<"Tiling (100%)"<<std::endl;

}
