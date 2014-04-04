////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include "mnt.hpp"
#include <string.h>
//#include "tga.h"

#ifdef _MSC_VER
#pragma warning(disable : 4996) // TEMP MT
#endif
////////////////////////////////////////////////////////////////////////////////
MNT::MNT()
{
	dim_x = 0;
	dim_y = 0;
	altitudes = 0;
	image = 0;
	mnt_charge = false;
}
////////////////////////////////////////////////////////////////////////////////
MNT::~MNT()
{
	if( altitudes )
		delete[] altitudes;

	if( image )
		delete[] image;
}
////////////////////////////////////////////////////////////////////////////////
bool MNT::charge( char* nom_fichier )
{
	FILE	*fp;
	char	chaine[500];

	if( altitudes )
		delete[] altitudes;

	if( image )
		delete[] image;

	mnt_charge = false;

	fp = fopen( nom_fichier, "rt" );
	if( !fp )
		return false;

	fscanf( fp, "%s", chaine );					// "MNT"
	if( strcmp( chaine, "MNT" ) != 0 )
		return false;

	fscanf( fp, "%s", chaine );					// Numéro de version
	fscanf( fp, "%s", nom_chantier );			// Nom du chantier
	fscanf( fp, "%s", unites_xy );				// Unité des xy
	fscanf( fp, "%f", &precision_xy );			// Précision de l'unité
	fscanf( fp, "%f", &x_noeud_NO );			// x du noeud Nord-Ouest
	fscanf( fp, "%f", &y_noeud_NO );			// y du noeud Nord-Ouest
	fscanf( fp, "%f", &pas_x );					// pas en x
	fscanf( fp, "%f", &pas_y );					// pas en y
	fscanf( fp, "%d", &dim_y );					// nombre de lignes
	fscanf( fp, "%d", &dim_x );					// nombre de colonnes
	fscanf( fp, "%s", unites_z );				// Unité des z
	fscanf( fp, "%f", &precision_z );			// Précision de l'unité


	y_noeud_NO -= 3000000;

	printf( "origine = (%f, %f)\n", x_noeud_NO, y_noeud_NO );

	altitudes = new int[dim_x*dim_y];

	int i, offset = 0;

	// Lecture des altitudes
	for( int y=0; y<dim_y; y++ )
	{
		for( int x=0; x<dim_x; x++ )
		{
			fscanf( fp, "%d", &altitudes[offset++] );
		}
		printf( "Chargement (%d%%)\r", (int)(y*100.0/dim_y) );
		fflush(stdout);
	}
	printf("Chargement OK    \n");

	fclose(fp);

	mnt_charge = true;


	image = new byte[dim_x*dim_y*3];
	byte	r,g,b;
	float	val;

	int	altitude_min=10000, altitude_max=-10000;

	for( i=0; i<dim_x*dim_y; i++ )
	{
		if( altitudes[i] != -9999 )
		{
			if( altitudes[i] < altitude_min )
				altitude_min = altitudes[i];

			if( altitudes[i] > altitude_max )
				altitude_max = altitudes[i];
		}
	}

	for( i=0; i<dim_x*dim_y; i++ )
	{
		if( altitudes[i] == -9999 )
		{
			r=0; g=0; b=255;
		}
		else
		{
			val = 255.0f*(float)(altitudes[i] + altitude_min)/(altitude_min+altitude_max);

			if( val < 0 )
				val = 0;

			if( val > 255 )
				val = 255;

			r = g = b = (byte)val;
		}

		image[i*3  ] = r;
		image[i*3+1] = g;
		image[i*3+2] = b;
	}


	return true;
}
////////////////////////////////////////////////////////////////////////////////
void MNT::sauve_log( char* nom_fichier_log, char* nom_fichier_tga )
{
	FILE	*fp;

	fp = fopen( nom_fichier_log, "wt" );

	fprintf( fp, "nom_chantier : %s\n", nom_chantier );
	fprintf( fp, "x_noeud_NO   : %f\n", x_noeud_NO );
	fprintf( fp, "y_noeud_NO   : %f\n", y_noeud_NO );
	fprintf( fp, "pas_x        : %f\n", pas_x );
	fprintf( fp, "pas_y        : %f\n", pas_y );
	fprintf( fp, "dim_x        : %d\n", dim_x );
	fprintf( fp, "dim_y        : %d\n", dim_y );

	fclose( fp );

	//save_tga( nom_fichier_tga, dim_x, dim_y, image );
}
////////////////////////////////////////////////////////////////////////////////
bool MNT::sauve_partie( char* nom_fichier, int xpos, int ypos, int nb_pt_x, int nb_pt_y )
{
	FILE	*fp;

	fp = fopen( nom_fichier, "wt" );
	if( !fp )
		return false;

	printf( "Decoupe partie (%d,%d) de (%d x %d)\n", xpos, ypos, nb_pt_x, nb_pt_y );

	fprintf( fp, "%d %d\n", nb_pt_x, nb_pt_y );

	for( int y=ypos; y<ypos+nb_pt_y; y++ )
	{
		for( int x=xpos; x<xpos+nb_pt_x; x++ )
		{
			fprintf( fp, "%d ", altitudes[x+y*dim_x] );
		}
		fprintf( fp, "\n" );
	}

	fclose( fp );

	return true;
}
////////////////////////////////////////////////////////////////////////////////
bool MNT::sauve_partie_XML( char* nom_fichier, int xpos, int ypos, int nb_pt_x, int nb_pt_y )
{
	FILE	*fp;

	fp = fopen( nom_fichier, "wt" );
	if( !fp )
		return false;
	printf( "origine du XML= (%f, %f)\n", x_noeud_NO, y_noeud_NO );
	printf( "Decoupe partie (%d,%d) de (%d x %d)\n", xpos, ypos, nb_pt_x, nb_pt_y );

	//
	/* Début de création du fichier XML
	<TERRAIN>
		<ORIGINE> Lon Lat </ORIGINE>
		<VALX> nb_Pt_x </VALX>
		<VALY> nb_Pt_y </VALY>
		<ALTITUDE> x y z </POINT>
		...
    </TERRAIN>
	*/

	fprintf( fp, "<origine> %f \t %f </origine> \n", x_noeud_NO+(pas_x * xpos), y_noeud_NO+(pas_y*ypos) );
	fprintf( fp, "<valx> %d </valx> \n <valy> %d </valy> \n", nb_pt_x, nb_pt_y );
	for( int y=ypos; y<ypos+nb_pt_y; y++ )
	{
		for( int x=xpos; x<xpos+nb_pt_x; x++ )
		{
			fprintf( fp, "<POINT> %f %f %d </POINT>\n", x_noeud_NO+(pas_x * x), y_noeud_NO+(pas_y*y), altitudes[x+y*dim_x] );
		}
		//fprintf( fp, "\n" );
	}

	fclose( fp );

	return true;
}
////////////////////////////////////////////////////////////////////////////////
