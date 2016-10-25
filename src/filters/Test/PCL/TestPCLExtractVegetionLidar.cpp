
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>
#include "ogrsf_frmts.h"

#include <lasreader.hpp>
#include <laswriter.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "citygml.hpp"
#include "export/exportCityGML.hpp"
#include "utils/OGRGDAL_Utils/OGRGDALtoShpWriter.hpp"
#include "tile.hpp"

/**
* @brief Extrait les zones composees de nombreux points proches
* @param cloud : nuage de points en entree
* @return un nuage de point avec les zones colorees selon les regroupements
*/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
SegmentationPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
   // Placeholder for the resulting segments:
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointsSegmentes;

   // Creating the KdTree object for the search method of the extraction
   pcl::search::KdTree<pcl::PointXYZ>::Ptr
      tree (new pcl::search::KdTree<pcl::PointXYZ>);
   tree->setInputCloud (cloud);

   std::vector<pcl::PointIndices> cluster_indices;
   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
   ec.setClusterTolerance ( 2 ); //1
   ec.setMinClusterSize ( 50 );
   ec.setMaxClusterSize ( 100000000 );
   ec.setSearchMethod ( tree );
   ec.setInputCloud ( cloud );
   ec.extract ( cluster_indices );

   for ( std::vector<pcl::PointIndices>::const_iterator
           it  = cluster_indices.begin ();
           it != cluster_indices.end ();
         ++it )
   {
      double r = 255 * rand() / (RAND_MAX + 1.0f);
      double g = 255 * rand() / (RAND_MAX + 1.0f);
      double b = 255 * rand() / (RAND_MAX + 1.0f);

      pcl::PointCloud<pcl::PointXYZ>::Ptr
         cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for ( std::vector<int>::const_iterator pit  = it->indices.begin ();
                                             pit != it->indices.end ();
                                           ++pit )
      {
         cloud_cluster->points.push_back (cloud->points[*pit]);
      }
      cloud_cluster->width = (int)cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      PointsSegmentes.push_back(cloud_cluster);
   }

   return PointsSegmentes;
}

unsigned int ClampIt(unsigned int x,unsigned int a,unsigned int b)
{
    return x < a ? a : (x > b ? b : x);
}

citygml::CityObject*
Create3DTriangulatedVegetationFromPointCloud(
                                pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud,
                                std::string name )
{
    OGRGeometry* Emprise = new OGRPolygon;

    const float cellSize = 2.0; //in meter
    unsigned int ii = 0;
    unsigned int jj = 0;

    //Get and store the bounding box of the cloud point
    float xmin = FLT_MAX;
    float ymin = FLT_MAX;
    float xmax = -FLT_MAX;
    float ymax = -FLT_MAX;

    for(pcl::PointXYZ p : Cloud->points)
    {
        xmin = std::min(xmin,p.x);
        ymin = std::min(ymin,p.y);
        xmax = std::max(xmax,p.x);
        ymax = std::max(ymax,p.y);
    }

    //Float dimension of box
    float widthtemp = ceil(xmax - xmin);
    float heighttemp = ceil(ymax - ymin);

    //How many cell we have
    unsigned int cellwidthcount = static_cast<unsigned int>(widthtemp/cellSize);
    unsigned int cellheightcount = static_cast<unsigned int>(heighttemp/cellSize);

    // Where heights are stored
    std::vector<std::vector<double>> table(cellwidthcount,std::vector<double>(cellheightcount,0.0));
    // Where how many heights we have stored in table
    std::vector<std::vector<unsigned int>> tableCount(cellwidthcount,std::vector<unsigned int>(cellheightcount,0));

    if(cellwidthcount > 0.0 && cellheightcount > 0.0)
    {
        for(pcl::PointXYZ p : Cloud->points)
        {
            unsigned int xt = ClampIt(static_cast<unsigned int>((p.x - xmin) / cellSize),0,cellwidthcount-1);
            unsigned int yt = ClampIt(static_cast<unsigned int>((p.y - ymin) / cellSize),0,cellheightcount-1);

            table[xt][yt] += p.z;
            tableCount[xt][yt]++;
        }

        for(unsigned int i = 0; i < cellwidthcount; i++)
        {
            for(unsigned int j = 0; j < cellheightcount; j++)
            {
                if(tableCount[i][j] > 0)
                {
                    table[i][j] = table[i][j] / tableCount[i][j];
                }
            }
        }

        citygml::Geometry* cityGeom = new citygml::Geometry(name+"_Geometry", citygml::GT_Unknown, 2);

        for(unsigned int i = 0; i < cellwidthcount - 1; i++)
        {
            for(unsigned int j = 0; j < cellheightcount - 1; j++)
            {
                double iReal = i*cellSize;
                double jReal = j*cellSize;
                double ipReal = (i+1)*cellSize;
                double jpReal = (j+1)*cellSize;

                if(table[i][j] > 0.0 && table[i+1][j] > 0.0 && table[i+1][j+1] > 0.0 && table[i][j+1] > 0.0)
                {
                    citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
                    ring->addVertex(TVec3d(iReal+xmin,jReal+ymin,table[i][j]));
                    ring->addVertex(TVec3d(ipReal+xmin,jReal+ymin,table[i+1][j]));
                    ring->addVertex(TVec3d(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
                    ring->addVertex(TVec3d(iReal+xmin,jpReal+ymin,table[i][j+1]));
                    citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
                    poly->addRing(ring);
                    cityGeom->addPolygon(poly);

                    OGRLinearRing* Ring = new OGRLinearRing;
                    Ring->addPoint(new OGRPoint(iReal+xmin,jReal+ymin,table[i][j]));
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jReal+ymin,table[i+1][j]));
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
                    Ring->addPoint(new OGRPoint(iReal+xmin,jpReal+ymin,table[i][j+1]));
                    Ring->addPoint(new OGRPoint(iReal+xmin,jReal+ymin,table[i][j]));

                    OGRPolygon* Poly = new OGRPolygon;
                    Poly->addRingDirectly(Ring);
                    OGRGeometry* tmp = Emprise;
                    Emprise = Poly->Union(tmp);
                    delete tmp;
                }
                else if(table[i][j] > 0.0 && table[i+1][j] > 0.0 && table[i+1][j+1] > 0.0 && table[i][j+1] == 0.0)
                {
                    citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
                    ring->addVertex(TVec3d(iReal+xmin,jReal+ymin,table[i][j]));
                    ring->addVertex(TVec3d(ipReal+xmin,jReal+ymin,table[i+1][j]));
                    ring->addVertex(TVec3d(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
                    citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
                    poly->addRing(ring);
                    cityGeom->addPolygon(poly);

                    OGRLinearRing* Ring = new OGRLinearRing;
                    Ring->addPoint(new OGRPoint(iReal+xmin,jReal+ymin,table[i][j]));
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jReal+ymin,table[i+1][j]));
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
                    Ring->addPoint(new OGRPoint(iReal+xmin,jReal+ymin,table[i][j]));

                    OGRPolygon* Poly = new OGRPolygon;
                    Poly->addRingDirectly(Ring);
                    OGRGeometry* tmp = Emprise;
                    Emprise = Poly->Union(tmp);
                    delete tmp;
                }
                else if(table[i][j] > 0.0 && table[i+1][j] > 0.0 && table[i+1][j+1] == 0.0 && table[i][j+1] > 0.0)
                {
                    citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
                    ring->addVertex(TVec3d(iReal+xmin,jReal+ymin,table[i][j]));
                    ring->addVertex(TVec3d(ipReal+xmin,jReal+ymin,table[i+1][j]));
                    ring->addVertex(TVec3d(iReal+xmin,jpReal+ymin,table[i][j+1]));
                    citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
                    poly->addRing(ring);
                    cityGeom->addPolygon(poly);

                    OGRLinearRing* Ring = new OGRLinearRing;
                    Ring->addPoint(new OGRPoint(iReal+xmin,jReal+ymin,table[i][j]));
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jReal+ymin,table[i+1][j]));
                    Ring->addPoint(new OGRPoint(iReal+xmin,jpReal+ymin,table[i][j+1]));
                    Ring->addPoint(new OGRPoint(iReal+xmin,jReal+ymin,table[i][j]));

                    OGRPolygon* Poly = new OGRPolygon;
                    Poly->addRingDirectly(Ring);
                    OGRGeometry* tmp = Emprise;
                    Emprise = Poly->Union(tmp);
                    delete tmp;
                }
                else if(table[i][j] > 0.0 && table[i+1][j] == 0.0 && table[i+1][j+1] > 0.0 && table[i][j+1] > 0.0)
                {
                    citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
                    ring->addVertex(TVec3d(iReal+xmin,jReal+ymin,table[i][j]));
                    ring->addVertex(TVec3d(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
                    ring->addVertex(TVec3d(iReal+xmin,jpReal+ymin,table[i][j+1]));
                    citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
                    poly->addRing(ring);
                    cityGeom->addPolygon(poly);

                    OGRLinearRing* Ring = new OGRLinearRing;
                    Ring->addPoint(new OGRPoint(iReal+xmin,jReal+ymin,table[i][j]));
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
                    Ring->addPoint(new OGRPoint(iReal+xmin,jpReal+ymin,table[i][j+1]));
                    Ring->addPoint(new OGRPoint(iReal+xmin,jReal+ymin,table[i][j]));

                    OGRPolygon* Poly = new OGRPolygon;
                    Poly->addRingDirectly(Ring);
                    OGRGeometry* tmp = Emprise;
                    Emprise = Poly->Union(tmp);
                    delete tmp;
                }
                else if(table[i][j] == 0.0 && table[i+1][j] > 0.0 && table[i+1][j+1] > 0.0 && table[i][j+1] > 0.0)
                {
                    citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_ring",true);
                    ring->addVertex(TVec3d(ipReal+xmin,jReal+ymin,table[i+1][j]));
                    ring->addVertex(TVec3d(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
                    ring->addVertex(TVec3d(iReal+xmin,jpReal+ymin,table[i][j+1]));
                    citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_poly");
                    poly->addRing(ring);
                    cityGeom->addPolygon(poly);

                    OGRLinearRing* Ring = new OGRLinearRing;
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jReal+ymin,table[i+1][j]));
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jpReal+ymin,table[i+1][j+1]));
                    Ring->addPoint(new OGRPoint(iReal+xmin,jpReal+ymin,table[i][j+1]));
                    Ring->addPoint(new OGRPoint(ipReal+xmin,jReal+ymin,table[i+1][j]));

                    OGRPolygon* Poly = new OGRPolygon;
                    Poly->addRingDirectly(Ring);
                    OGRGeometry* tmp = Emprise;
                    Emprise = Poly->Union(tmp);
                    delete tmp;
                }
            }
        }

        OGRMultiPolygon* EmpriseMP = new OGRMultiPolygon;

        if(Emprise->getGeometryType() == wkbPolygon25D)
            EmpriseMP->addGeometryDirectly(Emprise);
        else if(Emprise->getGeometryType() == wkbMultiPolygon25D)
            EmpriseMP = (OGRMultiPolygon*) Emprise;
        else
            delete Emprise;

        for(int i = 0; i < EmpriseMP->getNumGeometries(); ++i)
        {
            OGRPolygon* Poly = (OGRPolygon*) EmpriseMP->getGeometryRef(i);

            if(Poly == nullptr)
                continue;

            OGRLinearRing * ExtRing = Poly->getExteriorRing();

            if(ExtRing == nullptr)
                continue;

            for(int j = 0; j < ExtRing->getNumPoints(); ++j)
            {
                OGRPoint* P1 = new OGRPoint;
                OGRPoint* P2 = new OGRPoint;
                ExtRing->getPoint(j, P1);
                if(j < ExtRing->getNumPoints() - 1)
                    ExtRing->getPoint(j + 1, P2);
                else
                    ExtRing->getPoint(0, P2);

                citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_Verticalring",true);
                ring->addVertex(TVec3d(P1->getX(), P1->getY(), P1->getZ()));
                ring->addVertex(TVec3d(P2->getX(), P2->getY(), P2->getZ()));
                ring->addVertex(TVec3d(P1->getX(), P1->getY(), P1->getZ() - 6));
                citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_Verticalpoly");
                poly->addRing(ring);
                cityGeom->addPolygon(poly);

                citygml::LinearRing* ring2 = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_Verticalring",true);
                ring2->addVertex(TVec3d(P2->getX(), P2->getY(), P2->getZ()));
                ring2->addVertex(TVec3d(P2->getX(), P2->getY(), P2->getZ() - 6));
                ring2->addVertex(TVec3d(P1->getX(), P1->getY(), P1->getZ() - 6));
                citygml::Polygon* poly2 = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_Verticalpoly");
                poly2->addRing(ring2);
                cityGeom->addPolygon(poly2);
            }

            for(int k = 0; k < Poly->getNumInteriorRings(); ++k)
            {
                OGRLinearRing * IntRing = Poly->getInteriorRing(k);

                if(IntRing == nullptr)
                    continue;

                for(int j = 0; j < IntRing->getNumPoints(); ++j)
                {
                    OGRPoint* P1 = new OGRPoint;
                    OGRPoint* P2 = new OGRPoint;
                    IntRing->getPoint(j, P1);
                    if(j < IntRing->getNumPoints() - 1)
                        IntRing->getPoint(j + 1, P2);
                    else
                        IntRing->getPoint(0, P2);

                    citygml::LinearRing* ring = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_Verticalring",true);
                    ring->addVertex(TVec3d(P1->getX(), P1->getY(), P1->getZ()));
                    ring->addVertex(TVec3d(P2->getX(), P2->getY(), P2->getZ()));
                    ring->addVertex(TVec3d(P1->getX(), P1->getY(), P1->getZ() - 6)); //150
                    citygml::Polygon* poly = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_Verticalpoly");
                    poly->addRing(ring);
                    cityGeom->addPolygon(poly);

                    citygml::LinearRing* ring2 = new citygml::LinearRing(name+std::to_string(ii++)+std::to_string(jj++)+"_Verticalring",true);
                    ring2->addVertex(TVec3d(P2->getX(), P2->getY(), P2->getZ()));
                    ring2->addVertex(TVec3d(P2->getX(), P2->getY(), P2->getZ() - 6));
                    ring2->addVertex(TVec3d(P1->getX(), P1->getY(), P1->getZ() - 6));
                    citygml::Polygon* poly2 = new citygml::Polygon(name+std::to_string(ii++)+std::to_string(jj++)+"_Verticalpoly");
                    poly2->addRing(ring2);
                    cityGeom->addPolygon(poly2);
                }
            }
        }

        citygml::CityObject* VegetationCO;

        if(EmpriseMP->get_Area() > 100.0f) 
            VegetationCO = new citygml::PlantCover(name);
        else
            VegetationCO = new citygml::SolitaryVegetationObject(name);

        VegetationCO->addGeometry(cityGeom);

        delete EmpriseMP;

        return VegetationCO;
    }
    else
        return nullptr;

}

////////////////////////////////////////////////////////////////////////////////
std::string FilterVegetationFromLidar( std::string LiDAR_Path )
{
   std::string LiDAR_PathOutput = LiDAR_Path;

   LiDAR_PathOutput.insert(LiDAR_Path.find(".las"), "_Filtered");

   LASreadOpener lasreadopener;
   lasreadopener.set_file_name(LiDAR_Path.c_str());
   LASreader* lasreader = lasreadopener.open();
   if( !lasreader )
   {
     std::cout << "  LAS reader unable to open file "
               << LiDAR_Path 
               << std::endl;
     exit( EXIT_FAILURE );
   }
   std::cout << "  LAS reader opened." << std::endl;

   LASwriteOpener laswriteopener;
   laswriteopener.set_file_name(LiDAR_PathOutput.c_str());

   LASheader lasheader = lasreader->header;

   LASpoint laspoint;
   laspoint.init( &lasheader, lasheader.point_data_format,
                  lasheader.point_data_record_length, 0 );

   LASwriter* laswriter = laswriteopener.open(&lasheader);
   if( !laswriter )
   {
     std::cout << "  Unable to retrieve the laswriter." << std::endl;
     exit( EXIT_FAILURE );
   }
   std::cout << "  LAS writer opened." << std::endl;

   // Select the relevant points and write them down:
   while (lasreader->read_point())
   {
      // Only consider vegeration classes
      if( lasreader->point.classification != 3 && 
          lasreader->point.classification != 4 &&
          lasreader->point.classification != 5) 
      {
         continue;
      }

      // Weed out pulses that have a single return (they must correspond to
      // ground or buldings):
      if(lasreader->point.get_number_of_returns() <= 1) 
      {
         continue;
      }

      // Consider only the first return (that is the one corresponding
      // to the maximum height):
      if(lasreader->point.get_return_number() > 1) 
      {
         continue;
      }

      laswriter->write_point( &lasreader->point );
      laswriter->update_inventory( &laspoint );
   }

   laswriter->update_header( &lasheader, TRUE );

   I64 total_bytes = laswriter->close();

   std::cout << "  An intermediate LAS file ("
             << total_bytes << " bytes) was created here: " 
             << LiDAR_PathOutput << std::endl;

   delete laswriter;

   return LiDAR_PathOutput;
}

void Create3DVegetation( std::string LiDAR_Path,
                         std::string outputFileName )
{
   std::cout << "  Starting the computation of the 3D vegetation."
             << std::endl;

    /// Retrieve the LIDAR data as a PCL point cloud
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name( LiDAR_Path.c_str() );
    LASreader* LiDAR = lasreadopener.open();

    pcl::PointXYZ point;
    pcl::PointCloud< pcl::PointXYZ >::Ptr
       PointCloud (new pcl::PointCloud<pcl::PointXYZ>);

    while (LiDAR->read_point())
    {
        point.x = (LiDAR->point).get_x();
        point.y = (LiDAR->point).get_y();
        point.z = (LiDAR->point).get_z();

        PointCloud->points.push_back(point);
    }

    /// Realize a segmentation of the retrieved point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
       SegmentePointCloud = SegmentationPointCloud( PointCloud );

    citygml::CityModel* ModelOut = new citygml::CityModel;

    /// Convert each segment to a triangulation and append the
    /// result to the output citygml model
    int i = 1;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud : SegmentePointCloud)
    {
        citygml::CityObject* TrianguleCloud =
           Create3DTriangulatedVegetationFromPointCloud(
             Cloud,
             "Vegetation_" + std::to_string(i) );

        if(TrianguleCloud != nullptr)
        {
            ModelOut->addCityObject( TrianguleCloud );
            ModelOut->addCityObjectAsRoot( TrianguleCloud );
        }
        ++i;
    }

   /// Writing down the result as a CityGML file:
   std::cout << "  Writing the vegetation created content as CityGML file "
             << outputFileName << std::endl;
    citygml::ExporterCityGML exporter( outputFileName );
    ModelOut->computeEnvelope( );
    exporter.exportCityModel( *ModelOut );

    delete ModelOut;
}

void usage( int narg, char** argv )
{
  if ( narg != 3 )
  {
    std::cout << "  Wrong number of arguments. "            << std::endl 
              << "  Usage: " << argv[0]
              << "  lidar_data_filename citygml_filename" << std::endl
              << "  where the lidar data is the input information and citygml"
              << "  is the result of the filter."         << std::endl;
    exit( EXIT_FAILURE );
  }
}

////////////////////////////////////////////////////////////////////////////////
// The following "filter" illustrates how to extract some vegetation related
// out of lidar data
// Note: the lidar input data was previously filtered with IRC (Infra
// Red Camera) data but this process is left out of this example that focuses
// on using PCL

int main( int narg, char** argv )
{
   std::cout << "  Entering test " << argv[0] << std::endl;
   usage( narg, argv );
   std::string inputFileName  = argv[1];
   std::string outputFileName = argv[2];
   std::cout << "  Using file " << inputFileName  << " as input." << std::endl;
   std::cout << "  Using file " << outputFileName << " as output." << std::endl;

   std::cout << "  Entering FilterVegetationFromLidar." << std::endl;
   std::string LiDAR_Filtered_Path = FilterVegetationFromLidar( inputFileName );
   std::cout << "  FilterVegetationFromLidar done." << std::endl;

   Create3DVegetation( LiDAR_Filtered_Path, outputFileName );

   return EXIT_SUCCESS;
}

