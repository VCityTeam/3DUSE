#ifndef _IMPORTERASC_HPP_
#define _IMPORTERASC_HPP_

#include <queue>

#include "importer.hpp"
#include "../citygml.hpp"


#include "src/DataStructures/DEM/osgMnt.hpp"
#include "src/gui/osg/osgGDAL.hpp"

namespace citygml
{
  /**
   * This class houses several public methods for converting [ASCII Esri grids](https://en.wikipedia.org/wiki/Esri_grid) to CityGML.
   * Most of these methods are given one or several `MNT*` parameters and output a `citygml::CityObject*`
   */
  class ImporterASC : public Importer
  {
  public:
    ImporterASC(void);
    ~ImporterASC(void);

    /**
     * Converts the data given in a `MNT` object to a CityObject of type `TINRelief`. Calls `generateTriangles()` to create the geometries.
     */
    CityObject* reliefToCityGML(MNT*);

    /**
     * Converts the data given in a `MNT` object to a CityObject of type `WaterBody`. Calls `generateTriangles()` to create the geometries.
     * Maybe deprecated by `CityObject* waterToCityGMLPolygons(MNT*, float)`
     */
    CityObject* waterToCityGML(MNT*);

    /**
     * Converts the data given in a `MNT` object to a CityObject of type `WaterBody`. 
     * Used for water surfaces where a lot of neighboring points have the same elevation, which creates lots of redundancy in the data.  
     * It creates horizontal, square OGRPolygon for each data point, and if two neighboring points have the same elevation (more or less the precision given in param), it merges the corresponding polygons. OGRPolygons are then converted to CityGML.
     */
    CityObject* waterToCityGMLPolygons(MNT*, float);

    /**
     * This method converts the data given in two `MNT` objects to a CityObject of type `TINRelief`.  
     * Use for a tile where data exists at two different resolutions, but the more precise data isn't available everywhere.  
     * For each MNT, it creates list a of OGRPolygons in the same way as the `reliefToCityGML()` method. It then replaces the least precise polygons by the more precise ones when available, and realizes the seam at the border. OGRPolygons are then converted to CityGML.
     */
    CityObject* fusionResolutions(MNT* asc1, MNT* asc2);

  protected:

    /**
     * Creates a triangle regular network from a MNT object.  
     * Each elevation value of the MNT object is considered as the data for a point (and not for a square surface). Each cell of 4 points is used to create 2 triangles. 
     * Points having the "No Data" value defined in the MNT object are ignored.
     */
    Geometry* generateTriangles(MNT*);

    /**
     * Checks which points neighboring the one in front of the queue have the same elevation, and push them at the back of the queue.
     * Called in `waterToCityGMLPolygons()`.
     */
    void propagateCategory(MNT*, std::queue<std::pair<int, int>>*, float alt, float zPrec);

    /**
     * Creates a horizontal square OGRPolygon corresponding to the data point designed by x,y.
     * Called in `waterToCityGMLPolygons()`.
     */
    OGRPolygon* createPoly(MNT*, int x, int y, float prec);

    /**
     * Converts an `OGRPolygon` into a `citygml::Polygon`  
     * Note: This function may be a duplicate of `ConvertOGRPolytoGMLPoly()` in ToolAlgoCut (see Issue #104 on github ), but trying to use this last one resulted in crashes during `delete` operations.
     */
    Polygon* OGRPolyToGMLPoly(OGRPolygon*);

    /*
     * Array of bool that is used to store which points of the ASC raster have been treated.
     * Use in methods thant don't parse the raster sequentially (`waterToCityGMLPolygons()`)
     */
    bool* treated;
  };

} //namespace citygml

#endif //_IMPORTERASC_HPP_
