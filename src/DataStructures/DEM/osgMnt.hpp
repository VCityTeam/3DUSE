// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGMNT_HPP__
#define __OSGMNT_HPP__

#include <osg/Geode>
#include "vcitycore_export.h"

class VCITYCORE_EXPORT MNT
{
public:
    typedef unsigned char byte;

    MNT();
    MNT(int p_dim_x, int p_dim_y, float p_NODATA_value = -9999.0);
    ~MNT();

    bool charge(const char* nom_fichier, const char* type_fichier);
    bool write(const char* nom_fichier);
    unsigned int  get_numVertices() { return dim_x*dim_y; }
    float  get_altitude(const int x, const int y);
    osg::Vec3 get_normale(const int n) { return normales[n]; }
    osg::ref_ptr<osg::Geode> buildAltitudesGrid(float offset_x, float offset_y, float offset_z =/*5*/0.0f, int zfactor = 1);

    bool est_charge() { return mnt_charge; }
    int  get_dim_x() { return dim_x; }
    int  get_dim_y() { return dim_y; }
    float get_x_noeud_NO() { return x_noeud_NO; }
    float get_y_noeud_NO() { return y_noeud_NO; }
    float get_pas_x() { return pas_x; }
    float get_pas_y() { return pas_y; }
    float get_nodata() { return NODATA_value; }

    void set_x_noeud_NO(float p_x_noeud_NO) { x_noeud_NO = p_x_noeud_NO; }
    void set_y_noeud_NO(float p_y_noeud_NO) { y_noeud_NO = p_y_noeud_NO; }
    void set_pas_x(float p_pas_x) { pas_x = p_pas_x; }
    void set_pas_y(float p_pas_y) { pas_y = p_pas_y; }
    void set_nodata(float p_NODATA_value) { NODATA_value = p_NODATA_value; }
    bool set_altitude(int x, int y, float alt);

private:
    char	nom_chantier[500];
    char	unites_xy[20];
    float	precision_xy;
    int		dim_x, dim_y;
    float	x_noeud_NO, y_noeud_NO;
    float	pas_x, pas_y;
    char	unites_z[20];
    float	precision_z;

    float		NODATA_value;

    float		*altitudes;
    bool	mnt_charge;

    osg::Vec3	*normales;

public:
    byte	*image;
};
////////////////////////////////////////////////////////////////////////////////
#endif // __MNT_HPP__
