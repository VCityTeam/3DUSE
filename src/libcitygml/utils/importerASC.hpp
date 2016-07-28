#ifndef _IMPORTERASC_HPP_
#define _IMPORTERASC_HPP_

#include <queue>
#ifdef _MSC_VER
  // Inhibit dll-interface warnings concerning:
  // * gdal-1.11.4 internals (cpl_string.h)
  // * complains on STL _Id member (of class ImporterASC)
  # pragma warning(disable:4251)
#endif
#include <ogrsf_frmts.h>

#include "libcitygml/import/importer.hpp"
#include "libcitygml/citygml.hpp"
#include "DataStructures/DEM/osgMnt.hpp"
#include "citygmlutils_export.h"

namespace citygml
{
  /**
   * @brief Converter from MNT data (ASC files) to CityGML data
   *
   * @details This class houses several public methods for converting ASCII Esri grids to CityGML.
   * Most of these methods are given one or several `MNT*` parameters and output a `citygml::CityObject*`
   */
  class CITYGMLUTILS_EXPORT ImporterASC : public Importer
  {
  public:
    ImporterASC(void);
    ~ImporterASC(void);

    /**
     * @brief Converts the data given in a `MNT` object to a CityObject of type `TINRelief`.
     */
    CityObject* reliefToCityGML(MNT*);

    /**
     * @brief Converts the data given in a `MNT` object to a CityObject of type `WaterBody`.
     * @deprecated Maybe deprecated by `CityObject* waterToCityGMLPolygons(MNT*, float)`
     */
    CityObject* waterToCityGML(MNT*);

    /**
     * @brief Converts the data given in a `MNT` object to a CityObject of type `WaterBody`.
     *
     * @details Used for water surfaces where a lot of neighboring points 
     * have the same elevation, which creates lots of redundancy in the data.  
     * It creates horizontal, square OGRPolygon for each data point, 
     * and if two neighboring points have the same elevation (more or less the
     * precision given in param), it merges the corresponding polygons. 
     * OGRPolygons are then converted to CityGML.
     */
    CityObject* waterToCityGMLPolygons(MNT*, float);

    /**
     * @brief This method converts the data given in two `MNT` objects to a CityObject of type `TINRelief`.
     *
     * @details Use for a tile where data exists at two different resolutions,
     * but the more precise data isn't available everywhere.  
     * For each MNT, it creates list a of OGRPolygons in the same way as the 
     * `reliefToCityGML()` method. It then replaces the least precise polygons
     * by the more precise ones when available, and realizes the seam at the border.
     * OGRPolygons are then converted to CityGML.
     */
    CityObject* fusionResolutions(MNT* asc1, MNT* asc2);

  protected:

    /**
     * @brief Creates a triangle regular network from a MNT object.
     *
     * @details Each elevation value of the MNT object is considered as the data 
     * for a point (and not for a square surface). 
     * Each cell of 4 points is used to create 2 triangles. 
     * Points having the "No Data" value defined in the MNT object are ignored.
     */
    Geometry* generateTriangles(MNT*);

    /**
     * @brief Adds neighboring points with the same elevation to the queue
     * @details Checks which points neighboring the one in front of the queue
     * have the same elevation, and push them at the back of the queue.
     * Called in `waterToCityGMLPolygons()`.
     */
    void propagateCategory(MNT*, std::queue<std::pair<int, int>>*, float alt, float zPrec);

    /**
     * @brief Creates a horizontal square OGRPolygon corresponding to the data point designed by x,y.
     */
    OGRPolygon* createPoly(MNT*, int x, int y, float prec);

    /**
     * @brief Converts an `OGRPolygon` into a `citygml::Polygon`
     *
     * @remark This function may be a duplicate of `ConvertOGRPolytoGMLPoly()`
     * in ToolAlgoCut (see Issue #104 on github ), but trying to use ToolAlgoCut 
     * resulted in crashes during `delete` operations.
     */
    Polygon* OGRPolyToGMLPoly(OGRPolygon*);

    /*
     * @brief Array of bool that is used to store which points of the ASC raster have been treated.
     * Use in methods thant don't parse the raster sequentially (`waterToCityGMLPolygons()`)
     */
    bool* treated;
  };

} //namespace citygml

#endif //_IMPORTERASC_HPP_
