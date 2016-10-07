// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include <string>
#include <osg/Geometry>
#include "osgMnt.hpp"

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

    normales = 0;
}
////////////////////////////////////////////////////////////////////////////////
MNT::MNT(int p_dim_x, int p_dim_y, float p_NODATA_value)
{
    dim_x = p_dim_x;
    dim_y = p_dim_y;
    altitudes = new float[dim_x*dim_y];
    NODATA_value = p_NODATA_value;
    for (int i = 0; i < (dim_x*dim_y); i++) altitudes[i] = NODATA_value;

    image = 0;
    mnt_charge = false;

    normales = 0;
}
////////////////////////////////////////////////////////////////////////////////
bool MNT::set_altitude(int x, int y, float alt)
{
    if (x < dim_x && y < dim_y && altitudes)
    {
        altitudes[x + y*dim_x] = alt;
        return true;
    }
    else
        return false;
}
////////////////////////////////////////////////////////////////////////////////
MNT::~MNT()
{
    if (altitudes)
        delete[] altitudes;

    if (image)
        delete[] image;

    if (normales)
        delete[] normales;
}
////////////////////////////////////////////////////////////////////////////////
float  MNT::get_altitude(const int x, const int y)
{
    if (x < dim_x && y < dim_y)
        return altitudes[x + y*dim_x];
    else return NODATA_value;
}
////////////////////////////////////////////////////////////////////////////////
bool MNT::charge(const char* nom_fichier, const char* type_fichier)
{
    FILE	*fp;
    char	chaine[500];

    NODATA_value = -9999;

    if (altitudes)
        delete[] altitudes;

    if (image)
        delete[] image;

    if (normales)
        delete[] normales;

    mnt_charge = false;

    fp = fopen(nom_fichier, "rt");
    if (!fp)
        return false;

    if (strcmp(type_fichier, "MNT") == 0)
    {
        int r;

        r = fscanf(fp, "%s", chaine);				// "MNT"
        if (strcmp(chaine, "MNT") != 0)
            return false;

        r = fscanf(fp, "%s", chaine);				// Numero de version
        r = fscanf(fp, "%s", nom_chantier);			// Nom du chantier
        r = fscanf(fp, "%s", unites_xy);			// Unite des xy
        r = fscanf(fp, "%f", &precision_xy);			// Precision de l'unite
        r = fscanf(fp, "%f", &x_noeud_NO);			// x du noeud Nord-Ouest
        r = fscanf(fp, "%f", &y_noeud_NO);			// y du noeud Nord-Ouest
        r = fscanf(fp, "%f", &pas_x);				// pas en x
        r = fscanf(fp, "%f", &pas_y);				// pas en y
        r = fscanf(fp, "%d", &dim_y);				// nombre de lignes
        r = fscanf(fp, "%d", &dim_x);				// nombre de colonnes
        r = fscanf(fp, "%s", unites_z);			// Unite des z
        r = fscanf(fp, "%f", &precision_z);			// Precision de l'unite

        y_noeud_NO -= 3000000;
    }
    else if (strcmp(type_fichier, "ASC") == 0)
    {
        int r;

        strcpy(nom_chantier, "empty");

        r = fscanf(fp, "%s", chaine);
        if (strcmp(chaine, "ncols") == 0)
        {
            r = fscanf(fp, "%d", &dim_x);				// nombre de colonnes
            printf("ncols: %d\n", dim_x);
        }
        r = fscanf(fp, "%s", chaine);
        if (strcmp(chaine, "nrows") == 0)
        {
            r = fscanf(fp, "%d", &dim_y);				// nombre de lignes
            printf("nrows: %d\n", dim_y);
        }
        r = fscanf(fp, "%s", chaine);
        if (strcmp(chaine, "xllcorner") == 0)
        {
            r = fscanf(fp, "%f", &x_noeud_NO);			// x du noeud Nord-Ouest
            printf("xllcorner: %f\n", x_noeud_NO);
        }
        r = fscanf(fp, "%s", chaine);
        if (strcmp(chaine, "yllcorner") == 0)
        {
            r = fscanf(fp, "%f", &y_noeud_NO);			// y du noeud Nord-Ouest
            printf("yllcorner: %f\n", y_noeud_NO);
        }
        r = fscanf(fp, "%s", chaine);
        if (strcmp(chaine, "cellsize") == 0)
        {
            r = fscanf(fp, "%f", &pas_x);				// pas en x
            printf("cellsize: %f\n", pas_x);
            pas_y = pas_x;						// pas en y
        }
        else if ((strcmp(chaine, "dx") == 0))
        {
            r = fscanf(fp, "%f", &pas_x);				// pas en x
            printf("cellsize_x: %f\n", pas_x);

            r = fscanf(fp, "%s", chaine);
            r = fscanf(fp, "%f", &pas_y);				// pas en y
            printf("cellsize_y: %f\n", pas_y);
        }
        r = fscanf(fp, "%s", chaine);
        if (strcmp(chaine, "NODATA_value") == 0 || strcmp(chaine, "nodata_value") == 0)
        {
            r = fscanf(fp, "%f", &NODATA_value);			// NODATA_value
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

    printf("origine = (%f, %f)\n", x_noeud_NO, y_noeud_NO);

    altitudes = new float[dim_x*dim_y];
    normales = new osg::Vec3[dim_x*dim_y];

    // Lecture des altitudes
    int offset = 0;
    int r;
    for (int y = 0; y < dim_y; y++)
    {
        for (int x = 0; x < dim_x; x++)
        {
            float a = NODATA_value;
            r = fscanf(fp, "%f", &a);
            altitudes[offset] = a;
            normales[offset] = osg::Vec3(0.0, 0.0, 0.0);
            //printf("%d \n", altitudes[offset]);
            offset++;
        }
        printf("Chargement (%d%%)\r", (int)(y*100.0 / dim_y));
        fflush(stdout);
    }
    printf("Chargement OK    \n");

    fclose(fp);

    mnt_charge = true;

    // tga
    /*int i;
    image = new byte[dim_x*dim_y*3];
    byte	r,g,b;
    float	val;

    int	altitude_min=10000, altitude_max=-10000;

    for( i=0; i<dim_x*dim_y; i++ )
    {
        if( altitudes[i] != NODATA_value )
        {
            if( altitudes[i] < altitude_min )
                altitude_min = altitudes[i];

            if( altitudes[i] > altitude_max )
                altitude_max = altitudes[i];
        }
    }

    for( i=0; i<dim_x*dim_y; i++ )
    {
        if( altitudes[i] == NODATA_value )
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
    }*/

    // normals
    //unsigned int n=0;
    for (int y = 0; y < get_dim_y() - 1; y++)
        for (int x = 0; x < get_dim_x() - 1; x++)
        {
            osg::Vec3 v0, v1, v2;
            v0.set(x_noeud_NO + (pas_x * x), y_noeud_NO + (pas_y * y), get_altitude(x, y));
            v1.set(x_noeud_NO + (pas_x * (x + 1)), y_noeud_NO + (pas_y * y), get_altitude((x + 1), y));
            v2.set(x_noeud_NO + (pas_x * x), y_noeud_NO + (pas_y * (y + 1)), get_altitude(x, (y + 1)));

            // facet normal
            osg::Vec3 vector1, vector2, normal;
            vector1.set(v0 - v1);
            vector2.set(v1 - v2);
            normal = vector1 ^ vector2;  // cross product
            normal.normalize();

            // normal of the 3 vertices of the facet
            normales[x + y*dim_x] += normal;
            normales[(x + 1) + y*dim_x] += normal;
            normales[x + (y + 1)*dim_x] += normal;

            // ---

            osg::Vec3 v3;
            v3.set(x_noeud_NO + (pas_x * (x + 1)), y_noeud_NO + (pas_y * (y + 1)), get_altitude((x + 1), (y + 1)));

            // facet normal
            vector1.set(v1 - v3);
            vector2.set(v3 - v2);
            normal = vector1 ^ vector2;  // cross product
            normal.normalize();

            // normal of the 3 vertices of the facet
            normales[(x + 1) + y*dim_x] += normal;
            normales[(x + 1) + (y + 1)*dim_x] += normal;
            normales[x + (y + 1)*dim_x] += normal;
        }

    // normalize normal of the vertices
    for (int y = 0; y < get_dim_y(); y++)
        for (int x = 0; x < get_dim_x(); x++)
            normales[x + y*dim_x].normalize();

    return true;
}
////////////////////////////////////////////////////////////////////////////////
bool MNT::write(const char* nom_fichier)
{
    FILE* out;
    out = fopen(nom_fichier, "wt");
    fprintf(out, "ncols         %d\n", dim_x);
    fprintf(out, "nrows         %d\n", dim_y);
    fprintf(out, "xllcorner     %.3f\n", x_noeud_NO);
    fprintf(out, "yllcorner     %.3f\n", y_noeud_NO);
    fprintf(out, "cellsize      %.3f\n", pas_x);
    fprintf(out, "nodata_value  %.3f\n", NODATA_value);
    for (int y = 0; y < dim_y; y++)
    {
        for (int x = 0; x < dim_x; x++)
        {
            fprintf(out, "%.3f ", get_altitude(x, y));
        }
        fprintf(out, "\n");
    }

    fclose(out);
    return true;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> MNT::buildAltitudesGrid(float offset_x, float offset_y, float offset_z, int zfactor)
{
    osg::ref_ptr<osg::Geode> geode;
    geode = new osg::Geode;
    geode->setName("altitudesGrid");

    // Create geometry basic properties
    osg::Geometry* geom = new osg::Geometry;
    geode->addDrawable(geom);

    // vertices
    osg::Vec3Array* va = new osg::Vec3Array(get_numVertices());
    unsigned int i = 0;
    for (int y = 0; y < get_dim_y(); y++)
        for (int x = 0; x < get_dim_x(); x++)
            (*va)[i++].set(x_noeud_NO + offset_x + (pas_x * x), y_noeud_NO + (get_dim_y()*pas_y) + offset_y + (pas_y * -y), (get_altitude(x, y) - offset_z) * zfactor);
    geom->setVertexArray(va);

    // normals
    osg::Vec3Array* na = new osg::Vec3Array(get_numVertices());
    unsigned int n = 0;
    for (int y = 0; y < get_dim_y(); y++)
        for (int x = 0; x < get_dim_x(); x++)
        {
            (*na)[n] = get_normale(n);
            n++;
        }
    geom->setNormalArray(na);
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    // Create geometry primitives
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);

    for (int y = 0; y < get_dim_y() - 1; y++)
        for (int x = 0; x < get_dim_x() - 1; x++)
        {
            indices->push_back(y*get_dim_x() + x);
            indices->push_back(y*get_dim_x() + (x + 1));
            indices->push_back((y + 1)*get_dim_x() + x);

            indices->push_back(y*get_dim_x() + (x + 1));
            indices->push_back((y + 1)*get_dim_x() + (x + 1));
            indices->push_back((y + 1)*get_dim_x() + x);
        }

    geom->getOrCreateStateSet(); // MT : rajoute par MM -> utilite ?

    geom->addPrimitiveSet(indices);
    // Create geometry primitives

    return geode;
}
////////////////////////////////////////////////////////////////////////////////
