#include "importerASC.hpp"
#include "src/processes/ToolAlgoCut.hpp"

#include <QInputDialog>
#include <QDir>

namespace citygml
{

  ImporterASC::ImporterASC(void)
  {
  }

  ImporterASC::~ImporterASC(void)
  {
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  CityObject* ImporterASC::reliefToCityGML(MNT* asc)
  {
    CityObject* reliefTIN = new TINRelief("");

    reliefTIN->addGeometry(generateTriangles(asc));

    std::cout << "Conversion OK    " << std::endl;
    return reliefTIN;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  CityObject* ImporterASC::waterToCityGML(MNT* asc)
  {
    CityObject* waterbody = new WaterBody("");
    waterbody->addGeometry(generateTriangles(asc));
    std::cout << "Conversion OK    " << std::endl;
    return waterbody;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  Geometry* ImporterASC::generateTriangles(MNT* asc)
  {
    int incrx = 1; // for manually changing step if needed
    int incry = 1;
    Geometry* geom = new Geometry("", GT_Unknown, 3);
    for (int y = 0; y < asc->get_dim_y() - incry; y += incry)
    {
      for (int x = 0; x < asc->get_dim_x() - incrx; x += incrx)
      {
        //get the coordinates
        float xmin = (asc->get_x_noeud_NO()) + (x)*(asc->get_pas_x());
        float xmax = (asc->get_x_noeud_NO()) + (x + incrx)*(asc->get_pas_x());
        float ymax = (asc->get_y_noeud_NO()) + (asc->get_dim_y() - y - 1)*(asc->get_pas_y());
        float ymin = (asc->get_y_noeud_NO()) + (asc->get_dim_y() - y - 1 - incry)*(asc->get_pas_y());
        TVec3d v1, v2, v3, v4;
        v1[0] = xmin;
        v1[1] = ymax;
        v1[2] = asc->get_altitude(x, y);
        v2[0] = xmin;
        v2[1] = ymin;
        v2[2] = asc->get_altitude(x, y + incry);
        v3[0] = xmax;
        v3[1] = ymax;
        v3[2] = asc->get_altitude(x + incrx, y);
        v4[0] = xmax;
        v4[1] = ymin;
        v4[2] = asc->get_altitude(x + incrx, y + incry);

        if (v1[2] != (asc->get_nodata()) && v2[2] != (asc->get_nodata()) && v3[2] != (asc->get_nodata()))
        {
          Polygon* t = new Polygon("");
          LinearRing* lr = new LinearRing("", true);
          lr->getVertices().push_back(v1);
          lr->getVertices().push_back(v2);
          lr->getVertices().push_back(v3);
          t->addRing(lr);
          geom->addPolygon(t);
        }
        if (v2[2] != (asc->get_nodata()) && v3[2] != (asc->get_nodata()) && v4[2] != (asc->get_nodata()))
        {
          Polygon* t = new Polygon("");
          LinearRing* lr = new LinearRing("", true);
          lr->getVertices().push_back(v3);
          lr->getVertices().push_back(v2);
          lr->getVertices().push_back(v4);
          t->addRing(lr);
          geom->addPolygon(t);
        }
      }
      printf("Conversion (%d%%)\r", (int)(y*100.0 / asc->get_dim_y()));
      fflush(stdout);
    }
    return geom;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  CityObject* ImporterASC::waterToCityGMLPolygons(MNT* asc, float zPrec = 0.1)
  {
    CityObject* waterbody = new WaterBody("");

    treated = new bool[asc->get_dim_x()*asc->get_dim_y()];
    for (int i = 0; i < (asc->get_dim_x()*asc->get_dim_y()); i++) treated[i] = false;
    for (int y = 0; y < asc->get_dim_y(); y++)
    {
      for (int x = 0; x < asc->get_dim_x(); x++)
      {
        if (!treated[x + y*asc->get_dim_x()] && asc->get_altitude(x, y) != asc->get_nodata())
        {
          Geometry* geom = new Geometry("", GT_Unknown, 3);

          OGRMultiPolygon* pOgrMerged = new OGRMultiPolygon;
          std::queue<std::pair<int, int>> pointsList;
          pointsList.push(std::make_pair(x, y));
          treated[x + y*asc->get_dim_x()] = true;
          float alt = asc->get_altitude(x, y);
          // Search and create Polygons to merge
          while (!pointsList.empty())
          {
            pOgrMerged->addGeometryDirectly(createPoly(asc, pointsList.front().first, pointsList.front().second, 1 / zPrec));
            propagateCategory(asc, &pointsList, alt, zPrec);
            pointsList.pop();
          }
          // Merge Polygons
          OGRGeometry* pTemp = pOgrMerged->UnionCascaded();
          OGRPolygon* poPG = (OGRPolygon*)pTemp;
          if (poPG == nullptr)
          {
            std::cout << " ERREUR pOgrMerged nest pas un polygone ! " << poPG->getGeometryName() << std::endl;
            int a;
            std::cin >> a;
          }
          //convert merged Polygon to CityGML
          geom->addPolygon(OGRPolyToGMLPoly(poPG));

          if (geom->size() != 0)
          {
            waterbody->addGeometry(geom);
          }
          delete pOgrMerged;
          delete poPG;
        }
      }
      std::cout << "Constructing geometries (" << (int)(y*100.0 / asc->get_dim_y()) << "%)\r";
      fflush(stdout);
    }
    std::cout << "Constructing geometries OK!     " << std::endl;
    delete treated;

    return waterbody;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  void ImporterASC::propagateCategory(MNT* asc, std::queue<std::pair<int, int>>* pointsList, float alt, float prec = 1)
  {
    int x = pointsList->front().first;
    int y = pointsList->front().second;
    //std::cout<<x<<";"<<y<<" / "<<dim_x<<";"<<dim_y<<std::endl;

    // If some neighbours have same altitude but no category
    if (y > 0 && !treated[x + (y - 1)*asc->get_dim_x()] && std::abs(asc->get_altitude(x, y - 1) - alt) < prec)// up
    {
      pointsList->push(std::make_pair(x, y - 1));
      treated[x + (y - 1)*asc->get_dim_x()] = true;
    }
    if (x > 0 && !treated[x - 1 + y*asc->get_dim_x()] && std::abs(asc->get_altitude(x - 1, y) - alt) < prec)// left
    {
      pointsList->push(std::make_pair(x - 1, y));
      treated[x - 1 + y*asc->get_dim_x()] = true;
    }
    if ((y + 1) < asc->get_dim_y() && !treated[x + (y + 1)*asc->get_dim_x()] && std::abs(asc->get_altitude(x, y + 1) - alt) < prec)// down
    {
      pointsList->push(std::make_pair(x, y + 1));
      treated[x + (y + 1)*asc->get_dim_x()] = true;
    }
    if ((x + 1) < asc->get_dim_x() && !treated[x + 1 + y*asc->get_dim_x()] && std::abs(asc->get_altitude(x + 1, y) - alt) < prec)// right
    {
      pointsList->push(std::make_pair(x + 1, y));
      treated[x + 1 + y*asc->get_dim_x()] = true;
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  OGRPolygon* ImporterASC::createPoly(MNT* asc, int x, int y, float prec = 1)
  {
    OGRLinearRing* ring = new OGRLinearRing;
    // calculate coordinates, x & y correspond to the SW point (xmin, ymin)
    float xmin = asc->get_x_noeud_NO() + x*asc->get_pas_x();
    float ymax = asc->get_y_noeud_NO() + asc->get_dim_y() * asc->get_pas_y() - y * asc->get_pas_y();
    float xmax = asc->get_x_noeud_NO() + (x + 1)*asc->get_pas_x();
    float ymin = asc->get_y_noeud_NO() + asc->get_dim_y() * asc->get_pas_y() - (y + 1)*asc->get_pas_y();

    ring->addPoint(xmin, ymin, floor(asc->get_altitude(x, y)*prec + 0.5) / prec);
    ring->addPoint(xmin, ymax, floor(asc->get_altitude(x, y)*prec + 0.5) / prec);
    ring->addPoint(xmax, ymax, floor(asc->get_altitude(x, y)*prec + 0.5) / prec);
    ring->addPoint(xmax, ymin, floor(asc->get_altitude(x, y)*prec + 0.5) / prec);
    ring->addPoint(xmin, ymin, floor(asc->get_altitude(x, y)*prec + 0.5) / prec);

    OGRPolygon* poly = new OGRPolygon();
    poly->addRingDirectly(ring);
    return poly;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  Polygon* ImporterASC::OGRPolyToGMLPoly(OGRPolygon* orgPoly)
  {
    //get interior and exterior rings
    std::vector<TVec3d> extRing;
    OGRPoint p;
    TVec3d v;
    OGRLinearRing* poLR = orgPoly->getExteriorRing();
    if (poLR == nullptr) return nullptr;
    for (int i = 0; i < poLR->getNumPoints(); ++i)
    {
      poLR->getPoint(i, &p);
      v = TVec3d(p.getX(), p.getY(), p.getZ());
      extRing.push_back(v);
    }
    std::vector<std::vector<TVec3d>> intRing;

    for (unsigned int i = 0; i < (unsigned int)orgPoly->getNumInteriorRings(); i++)
    {
      std::vector<TVec3d> intRingTemp;
      poLR = orgPoly->getInteriorRing(i);
      for (int i = 0; i < poLR->getNumPoints(); ++i)
      {
        poLR->getPoint(i, &p);
        v = TVec3d(p.getX(), p.getY(), p.getZ());
        intRingTemp.push_back(v);
      }
      if (intRingTemp.size() == 0)
        continue;
      intRing.push_back(intRingTemp);

    }
    //build CityGML polygon
    citygml::Polygon* GMLpoly = new citygml::Polygon("");
    citygml::LinearRing* ring1 = new citygml::LinearRing("", true);
    for (unsigned int j = 0; j < extRing.size(); j++)
    {
      ring1->addVertex(extRing[j]);
    }
    GMLpoly->addRing(ring1);
    for (std::vector<TVec3d> vec : intRing)
    {
      citygml::LinearRing* ring2 = new citygml::LinearRing("", false);
      for (unsigned int j = 0; j < vec.size(); j++)
      {
        ring2->addVertex(vec[j]);
      }
      GMLpoly->addRing(ring2);
    }
    return GMLpoly;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  CityObject* ImporterASC::fusionResolutions(MNT* asc1, MNT* asc2)
  {
    const int prec = 10;
    //create OGRPloygons for both asc
    std::vector<OGRPolygon*> polys1, polys2;
    //first asc
    for (int y = 0; y < asc1->get_dim_y() - 1; y++)
    {
      for (int x = 0; x < asc1->get_dim_x() - 1; x++)
      {
        float xmin = (asc1->get_x_noeud_NO()) + (x)*(asc1->get_pas_x());
        float xmax = (asc1->get_x_noeud_NO()) + (x + 1)*(asc1->get_pas_x());
        float ymax = (asc1->get_y_noeud_NO()) + (asc1->get_dim_y() - y - 1)*(asc1->get_pas_y());
        float ymin = (asc1->get_y_noeud_NO()) + (asc1->get_dim_y() - y - 2)*(asc1->get_pas_y());
        //1st triangle
        if (asc1->get_altitude(x, y) != asc1->get_nodata() && asc1->get_altitude(x, y + 1) != asc1->get_nodata() && asc1->get_altitude(x + 1, y) != asc1->get_nodata())
        {
          OGRLinearRing* ring1 = new OGRLinearRing;
          ring1->addPoint(xmin, ymax, floor(asc1->get_altitude(x, y)*prec + 0.5) / prec);
          ring1->addPoint(xmin, ymin, floor(asc1->get_altitude(x, y + 1)*prec + 0.5) / prec);
          ring1->addPoint(xmax, ymax, floor(asc1->get_altitude(x + 1, y)*prec + 0.5) / prec);
          ring1->addPoint(xmin, ymax, floor(asc1->get_altitude(x, y)*prec + 0.5) / prec);
          OGRPolygon* poly1 = new OGRPolygon();
          poly1->addRingDirectly(ring1);
          polys1.push_back(poly1);
        }

        //2nd triangle
        if (asc1->get_altitude(x + 1, y) != asc1->get_nodata() && asc1->get_altitude(x, y + 1) != asc1->get_nodata() && asc1->get_altitude(x + 1, y + 1) != asc1->get_nodata())
        {
          OGRLinearRing* ring2 = new OGRLinearRing;
          ring2->addPoint(xmax, ymax, floor(asc1->get_altitude(x + 1, y)*prec + 0.5) / prec);
          ring2->addPoint(xmin, ymin, floor(asc1->get_altitude(x, y + 1)*prec + 0.5) / prec);
          ring2->addPoint(xmax, ymin, floor(asc1->get_altitude(x + 1, y + 1)*prec + 0.5) / prec);
          ring2->addPoint(xmax, ymax, floor(asc1->get_altitude(x + 1, y)*prec + 0.5) / prec);
          OGRPolygon* poly2 = new OGRPolygon();
          poly2->addRingDirectly(ring2);
          polys1.push_back(poly2);
        }
      }
      std::cout << "Building file 1 (" << (int)(y*100.0 / asc1->get_dim_y()) << "%)\r";
    }
    std::cout << "Building file 1 (100%)" << std::endl;
    //second asc
    for (int y = 0; y < asc2->get_dim_y() - 1; y++)
    {
      for (int x = 0; x < asc2->get_dim_x() - 1; x++)
      {
        float xmin = (asc2->get_x_noeud_NO()) + (x)*(asc2->get_pas_x());
        float xmax = (asc2->get_x_noeud_NO()) + (x + 1)*(asc2->get_pas_x());
        float ymax = (asc2->get_y_noeud_NO()) + (asc2->get_dim_y() - y - 1)*(asc2->get_pas_y());
        float ymin = (asc2->get_y_noeud_NO()) + (asc2->get_dim_y() - y - 2)*(asc2->get_pas_y());
        //1st triangle
        if (asc2->get_altitude(x, y) != asc2->get_nodata() && asc2->get_altitude(x, y + 1) != asc2->get_nodata() && asc2->get_altitude(x + 1, y) != asc2->get_nodata())
        {
          OGRLinearRing* ring1 = new OGRLinearRing;
          ring1->addPoint(xmin, ymax, floor(asc2->get_altitude(x, y)*prec + 0.5) / prec);
          ring1->addPoint(xmin, ymin, floor(asc2->get_altitude(x, y + 1)*prec + 0.5) / prec);
          ring1->addPoint(xmax, ymax, floor(asc2->get_altitude(x + 1, y)*prec + 0.5) / prec);
          ring1->addPoint(xmin, ymax, floor(asc2->get_altitude(x, y)*prec + 0.5) / prec);
          OGRPolygon* poly1 = new OGRPolygon();
          poly1->addRingDirectly(ring1);
          polys2.push_back(poly1);
        }
        //2nd triangle
        if (asc2->get_altitude(x + 1, y) != asc2->get_nodata() && asc2->get_altitude(x, y + 1) != asc2->get_nodata() && asc2->get_altitude(x + 1, y + 1) != asc2->get_nodata())
        {
          OGRLinearRing* ring2 = new OGRLinearRing;
          ring2->addPoint(xmax, ymax, floor(asc2->get_altitude(x + 1, y)*prec + 0.5) / prec);
          ring2->addPoint(xmin, ymin, floor(asc2->get_altitude(x, y + 1)*prec + 0.5) / prec);
          ring2->addPoint(xmax, ymin, floor(asc2->get_altitude(x + 1, y + 1)*prec + 0.5) / prec);
          ring2->addPoint(xmax, ymax, floor(asc2->get_altitude(x + 1, y)*prec + 0.5) / prec);
          OGRPolygon* poly2 = new OGRPolygon();
          poly2->addRingDirectly(ring2);
          polys2.push_back(poly2);
        }
      }
      std::cout << "Building file 2 (" << (int)(y*100.0 / asc2->get_dim_y()) << "%)\r";
    }
    std::cout << "Building file 2 (100%)" << std::endl;
    //Merging
    std::vector<OGRPolygon*> polysMerged;
    std::cout << "Merging...\r";
    int i = 1;
    for (OGRPolygon* poly1 : polys1)
    {
      std::list<OGRPolygon*> intersectingPolys;
      OGRPolygon* pRes = new OGRPolygon(*poly1);
      for (OGRPolygon* poly2 : polys2)
      {
        if (poly1->Intersects(poly2)) //for each least precise polygon, check if a more precise covers it
        {
          OGRPolygon* ptemp = (OGRPolygon*)pRes->Difference(poly2); //if yes, we substract the covered part
          pRes = ptemp;
          if (pRes == NULL) break;
          intersectingPolys.push_back(poly2);
        }
      }
      if (pRes != NULL && pRes->getExteriorRing() != nullptr && pRes->getExteriorRing()->getNumPoints()>2)
      { //some polys on the seam may have a Z coordinate no corrected during the 'Difference' operation, we fix this here
        for (int j = 0; j < pRes->getExteriorRing()->getNumPoints(); j++)
        {
          int npts = pRes->getExteriorRing()->getNumPoints();
          OGRPoint* pt = new OGRPoint();
          pRes->getExteriorRing()->getPoint(j, pt);
          for (OGRPolygon* iPoly : intersectingPolys)
          {
            if (pt->Touches(iPoly))
            {
              OGRPoint* newPt = ProjectPointOnPolygon3D(pt, iPoly);
              pRes->getExteriorRing()->setPoint(j, newPt->getX(), newPt->getY(), newPt->getZ());
              break;
            }
          }
        }
        polysMerged.push_back(pRes);
      }
      std::cout << "Merging (" << (int)(i++*100.0 / (polys1.size() + 1)) << "%)\r";
    }
    std::cout << "Merging (100%)" << std::endl;
    for (OGRPolygon* poly : polys2)
      polysMerged.push_back(poly);
    //Convert OGRPolygons to CityGML Geometry
    Geometry* geom = new Geometry("", GT_Unknown, 3);
    i = 1;
    for (OGRPolygon* poly : polysMerged)
    {
      Polygon* GMLPoly = OGRPolyToGMLPoly(poly);
      if (GMLPoly != nullptr) geom->addPolygon(GMLPoly);
      std::cout << "Conversion (" << (int)(i++*100.0 / (polysMerged.size() + 1)) << "%)\r";
    }
    //add to CityObject
    CityObject* reliefTIN = new TINRelief("");

    reliefTIN->addGeometry(geom);

    std::cout << "Conversion (100%)" << std::endl;

    return reliefTIN;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
}//namespace citygml
