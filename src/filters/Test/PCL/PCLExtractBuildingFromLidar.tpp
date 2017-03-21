#ifndef __PCLEXTRACTBUILDINGFROMLIDAR_TXX__
#define __PCLEXTRACTBUILDINGFROMLIDAR_TXX__

#include <iostream>
#include <fstream>
#include <tuple>

#if defined _MSC_VER
// Inhibiting complains about fopen and strcpy as being unsafe functions
// (called from flann-1.8.4\include\flann\... )
#define _CRT_SECURE_NO_DEPRECATE
#pragma warning (disable: 4267) // For lasdefinitions.hpp
#pragma warning (disable: 4996) // Mostly for lasdefinitions.hpp but also
                                // flann/util includes
#endif

// LASLIB for reading LIDAR (las) files
#include "lasreader.hpp"

// Point cloud library
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/normal_3d.h>   // For pcl::NormalEstimation
#include <pcl/surface/gp3.h>          // For pcl::GreedyProjectionTriangulation

namespace vcity
{
namespace testing
{

// Condition function of pcl::ConditionalEuclideanClustering()
// clustering algorithm.
template< typename PointT >
bool GrowAlongBuildingFacade( const PointT& pointA,
                              const PointT& pointB,
                              float squaredDistance )
{
   // Close enough points belong to the same cluster:
   if( squaredDistance < 2.0 )
   {
      return (true);
   }

   // If the candidate point is "above" or "below" the seed point we
   // then consider it as part of the cluster. "Above or below" is here
   // defined as lying within a vertical axis cone centered at the seed
   // point and with a prescribed angle (or slope):
   float horizontalSquaredDist = 
       ( pointA.x - pointB.x ) * ( pointA.x - pointB.x )
     + ( pointA.y - pointB.y ) * ( pointA.y - pointB.y );

   if( horizontalSquaredDist / squaredDistance  < 0.01 )
   {
      return (true);
   }

   return (false);
}

///////////////////////////
// LASLIB uses "double" as floating point numbers representation.
// By default PCL uses "float" (refer toe e.g. pcl::PointXYZ documentation).
// In order to manipulate geo-data with PCL a precision lossless strategy
// consists in doing a translation of all the data coordinates with an
// offset related to the center of the bounding box of the original data.
// After such a translation the coordinates are left with the meaningfull
// part of their information that is their relative position within the
// local (new) system of coordinates.
// @return: the building point cloud, the ground point cloud
template< typename TPointCloud >
std::tuple< typename TPointCloud::Ptr, typename TPointCloud::Ptr, float, float >
ReadAndOffsetLidarFile( boost::filesystem::path& lidarPath )
{
   typedef          TPointCloud       PCType;
   typedef typename PCType::Ptr       PCTypePtr;
   typedef typename PCType::PointType PointType;

   // Open and read the lidar las file:
   LASreadOpener lasreadopener;
   lasreadopener.set_file_name( lidarPath.string().c_str() );
   LASreader* lasreader = lasreadopener.open();
   if( !lasreader )
   {
      std::cerr << "LASLIB unable to read file " 
                << lidarPath << std::endl;
      std::cerr << "Exiting." << std::endl;
      exit(EXIT_FAILURE);
   }

   // For the offset strategy to be lossless, we must still assert
   // that the bounding box diameter is smaller than what float can represent:
   double widthX = lasreader->header.min_x + lasreader->header.max_x;
   double widthY = lasreader->header.min_y + lasreader->header.max_y;
   if(    widthX > std::numeric_limits< float >::max()
       || widthY > std::numeric_limits< float >::max() )
   {
      std::cerr << "Point cloud diameter too big for PCL to handle ."
                << "Exiting." << std::endl;
      exit(EXIT_FAILURE);
   }
  
   float offsetX = widthX / 2.0;
   float offsetY = widthY / 2.0;

   // Read all the points, retaining the buildings and ground ones
   // while offseting their coordinates within the bounding box.
   PCTypePtr buildingCloud( new PCType );
   PCTypePtr groundCloud(   new PCType );
   PointType point;
   while (lasreader->read_point())
   {
      point.x = lasreader->point.get_x() - offsetX;
      point.y = lasreader->point.get_y() - offsetY;
      point.z = lasreader->point.get_z();

      // Extract the buildings: refer to classifications of LAS1.4 table 9 of 
      // http://www.asprs.org/a/society/committees/standards/LAS_1_4_r13.pdf
      if( lasreader->point.classification == 6  )
      {
         buildingCloud->points.push_back( point );
      }

      // Extract the ground: 
      if( lasreader->point.classification == 2 )
      {
         groundCloud->points.push_back( point );
      }
   }
   return std::make_tuple( buildingCloud, groundCloud, offsetX, offsetY );
}

// Fit a plane (with a default ransac method) through the cloud of
// points and return its altitude
template< typename TPointCloud >
Eigen::VectorXf
ComputeAltitudeFittingPlane( typename TPointCloud::Ptr groundCloud )
{
   typedef typename TPointCloud::PointType PointType;
   typedef typename pcl::SampleConsensusModelPlane< PointType > ModelPlaneType;

   typename ModelPlaneType::Ptr model_p( new ModelPlaneType( groundCloud ) );
   pcl::RandomSampleConsensus< PointType > ransac( model_p );
   ransac.setDistanceThreshold( 0.1 );
   ransac.computeModel();

   Eigen::VectorXf groundPlaneCoefficients;
   ransac.getModelCoefficients( groundPlaneCoefficients );

   return groundPlaneCoefficients;
}

// @brief Remove the points that has a negative distance to the plane
//        provided as argument
// @return a cloud olding only the point above altitude
// Reference code: http://www.pointclouds.org/documentation/tutorials/extract_indices.php
template< typename TPointCloud >
typename TPointCloud::Ptr
RemoveUndergroundPoints( typename TPointCloud::Ptr inCloud,
                         Eigen::VectorXf planeCoefficients )
{
   typedef          TPointCloud       PCType;
   typedef typename PCType::Ptr       PCTypePtr;
   typedef typename PCType::PointType PointType;

   // Store the indexes of the points below the fitted plane:
   pcl::PointIndices::Ptr toWeedOut( new pcl::PointIndices() );
   for( int i = 0; i < inCloud->size (); i++ )
   {
      double altitude =
         pcl::pointToPlaneDistanceSigned( inCloud->points[i],
                                          planeCoefficients ); 
      if( altitude < 0 )
      {
         toWeedOut->indices.push_back( i );
      }
   }

   PCTypePtr outCloud( new PCType );
   // Initializing with true will allow the extraction:
   pcl::ExtractIndices< PointType > filter( true );
   filter.setInputCloud( inCloud );
   filter.setIndices( toWeedOut );
   filter.filter ( *outCloud);

   std::cerr << "  Initial number of points: " 
             << inCloud->points.size() << std::endl;
   size_t numberRemoved = inCloud->points.size() - outCloud->points.size();
   std::cerr << "  Number of disregarded points below ground level: " 
             << numberRemoved
             << " ( " << (float)numberRemoved / inCloud->size() << "%)"
             << std::endl;
   return( outCloud );
}

// @brief   Extract a triangle mesh out of cloud point of a building
// @return  A cloud holding only the point above altitude
// Reference code: http://pointclouds.org/documentation/tutorials/greedy_projection.php
template< typename TPointCloud >
pcl::PolygonMesh::Ptr
ReconstructBuildingEnveloppe( typename TPointCloud::Ptr buildingCloud,
                              Eigen::VectorXf planeCoefficients )
{
   typedef          TPointCloud        PCType;
   typedef typename PCType::PointType  PointType;

   // Up to this stage, the buildings point clouds are dense for the roofs
   // and for the facades that got lit by the LIDAR. Nevertheless some
   // facades are devoid of any points and some of them have few scaarce point
   // when getting close to be bottom of the facade. In order to retrieve
   // complete facades (so the building touches the ground) we duplicate 
   // all point cloud with the projections of those points on the ground,
   // prior to the extraction of the triangles (surface enveloppe).

   double altitude = planeCoefficients[ 3 ];
   int numPointsToDuplicate = buildingCloud->size();
   for( int i = 0; i < numPointsToDuplicate; i++ )
   {  
      PointType projected( buildingCloud->points[i].x,
                           buildingCloud->points[i].y,
                           altitude);
      buildingCloud->push_back( projected );
   }

   // Reconstruction requires a point cloud with normals
   typedef pcl::PointCloud< pcl::Normal >    NormalCloudType;
   typedef pcl::search::KdTree< PointType >  KdTreeType;
   pcl::NormalEstimation< PointType, pcl::Normal > normalEstimator;
   NormalCloudType::Ptr  buildingNormals( new NormalCloudType );
   typename KdTreeType::Ptr tree( new KdTreeType );
   tree->setInputCloud( buildingCloud );
   normalEstimator.setInputCloud( buildingCloud );
   normalEstimator.setSearchMethod( tree );
   normalEstimator.setKSearch( 20 );
   normalEstimator.compute( *buildingNormals );

   // Concatenate the points with their normals i.e.
   //   buildingWithNormals = buildingCloud "+" buildingNormals
   typedef pcl::PointCloud< pcl::PointNormal > PointNormalCloudType;
   PointNormalCloudType::Ptr buildingWithNormals( new PointNormalCloudType );
   pcl::concatenateFields( *buildingCloud, 
                           *buildingNormals,
                           *buildingWithNormals);

   // Prepare elements for Greedy Projection algorithm.
   // Create search tree.
   typedef pcl::search::KdTree< pcl::PointNormal > KdTreePointNormalType;
   KdTreePointNormalType::Ptr treeNormal( new KdTreePointNormalType );
   treeNormal->setInputCloud( buildingWithNormals );

   // Initialize objects
   pcl::GreedyProjectionTriangulation< pcl::PointNormal > GreedyProj;
   pcl::PolygonMesh::Ptr triangles( new pcl::PolygonMesh );

   // The maximum distance between connected points (maximum edge length)
   GreedyProj.setSearchRadius( 20. );

   // Set typical values for the parameters
   GreedyProj.setMu( 10.5 );
   GreedyProj.setMaximumNearestNeighbors( 100 );
   GreedyProj.setMaximumSurfaceAngle( M_PI/2 );   // 90 degrees
   GreedyProj.setMinimumAngle( M_PI/18 );         // 10 degrees
   GreedyProj.setMaximumAngle( 2*M_PI/3 );        // 120 degrees
   GreedyProj.setNormalConsistency( false );

   // Proceed with reconstruction
   GreedyProj.setInputCloud( buildingWithNormals );
   GreedyProj.setSearchMethod( treeNormal );
   GreedyProj.reconstruct( *triangles );

   return triangles;
}

/**
 * @brief Segment (with pcl's Conditional Euclidean Clustering algorithm)
 *        the incoming cloud into (proximity based) clusters each of which
 *        representing a building. 
 * @param[in] buildingCloud The incoming cloud holding all the buildings and
 *        that must be segmented
 * @param[in] debug The function will display debugging messages (to cerr)
 *        when true. No message gets displayed when false.
 * @return  The set of point clouds each of which representing a single building
 * @sa    Refer to
 *        <a href="http://pointclouds.org/documentation/tutorials/greedy_projection.php">PCL's greedy projection tutorial</a>
 *        for original tutorial code
 */
template< typename TPointCloud >
std::vector< typename TPointCloud::Ptr,
             Eigen::aligned_allocator< typename TPointCloud::Ptr > >
SegmentBuildingClouds( typename TPointCloud::Ptr buildingCloud,
                       bool debug = false )
{
   typedef          TPointCloud        PCType;
   typedef typename PCType::Ptr        PCPointerType;
   typedef typename PCType::PointType  PointType;
   typedef          std::vector< PCPointerType,
                                 Eigen::aligned_allocator< PCPointerType > >
                                       ResultType;

   // Set up a Conditional Euclidean Clustering class context:
   pcl::IndicesClustersPtr clusters(        new pcl::IndicesClusters );

   pcl::ConditionalEuclideanClustering< PointType > cec( true );
   cec.setInputCloud( buildingCloud );
   cec.setConditionFunction( &GrowAlongBuildingFacade< PointType > );
   // Radius for the k-NN searching, used to find the candidate points:
   cec.setClusterTolerance( 4.0 ); 
   // Clusters that make up less than 0.1% of the cloud’s total points
   // are considered too small to be buildings
   cec.setMinClusterSize( buildingCloud->points.size () / 1000 );
   // Clusters can be (almost) the size of the cloud if there is a single
   // building:
   cec.setMaxClusterSize( buildingCloud->points.size () );
   cec.segment( *clusters );

   if( debug ) {
      pcl::IndicesClustersPtr small_clusters ( new pcl::IndicesClusters );
      pcl::IndicesClustersPtr large_clusters ( new pcl::IndicesClusters );
      cec.getRemovedClusters( small_clusters, large_clusters );
      std::cerr << "    Disregarded clusters: " 
                << small_clusters->size() << " too small, "
                << large_clusters->size() << " too large." << std::endl;
   }

   // The set of resulting extracted buildings represented as point clouds:
   ResultType buildingClouds;
   for( int i = 0; i < clusters->size (); i++ )
   {
      // Extract the current cluster as an independent cloud point:
      PCPointerType newBuildingCloud( new PCType );
      pcl::copyPointCloud( *buildingCloud, 
                           (*clusters)[i].indices,
                           *newBuildingCloud );
      buildingClouds.push_back( newBuildingCloud );
   }
   return buildingClouds;
}

/////////////////////////
template< typename TPointCloud >
std::tuple< 
   std::vector< typename TPointCloud::Ptr,
                Eigen::aligned_allocator< typename TPointCloud::Ptr > >,
   std::vector< pcl::PolygonMesh::Ptr,
                Eigen::aligned_allocator< typename pcl::PolygonMesh::Ptr > >
>
ExtractBuildingsPointCloudsAndMeshes( boost::filesystem::path& lidarPath )
{
   std::cerr << "  Using file " <<   lidarPath << " as input."  << std::endl;

   //////////////////////////////////////////////////////////////////
   // Prepare "global" context
   typedef pcl::PointXYZ                PointType;
   typedef pcl::PointCloud< PointType > PCType;
   typedef PCType::Ptr                  PCPointerType;
   
   float offsetX;
   float offsetY;
   /// The point cloud corresponding to LIDAR detected ground 
   PCPointerType groundCloud(   new PCType );
   /// The point cloud corresponding to buildings:
   PCPointerType buildingCloud( new PCType );

   //////////////////////////////////////////////////////////////////
   // The data is noised with points that appear way below the ground
   // level. Proceed with removal of such "underground" points:
   //   1/ consider the points labeled as ground
   //   2/ fit a plane (with a default ransac method)
   //   3/ disregard the points above this plane

   // Stage 1: consider the points labeled as ground (set aside the buildings
   //          for the time building)
   std::tie( buildingCloud, groundCloud, offsetX, offsetY ) 
      = ReadAndOffsetLidarFile< PCType >( lidarPath );

   // Stage 2: fit a plane
   std::cerr << "  Computing ground fitting plane. ";
   Eigen::VectorXf planeCoefficients
      = ComputeAltitudeFittingPlane< PCType >( groundCloud );
   std::cerr << "Done." << std::endl;
   std::cerr << "    Ground fitted plane normal is: " 
             << planeCoefficients[0] << ", " 
             << planeCoefficients[1] << ", " 
             << planeCoefficients[2] << "." << std::endl;
   std::cerr << "    Ground fitted plane height (d coefficeint) is: "
             << planeCoefficients[3] << std::endl;

   // Stage 3: disregard the points above this plane
   // PCL uses boost smart pointers: the "old" PC gets automatically freed 
   buildingCloud = RemoveUndergroundPoints< PCType >( buildingCloud,
                                                      planeCoefficients );
   
   ////////////////////////////////////////////////////////////////
   // Retrieve individual buildings (as clouds) through segmentation:
   std::cerr << "  Starting segmentation (Conditional Euclidean Clustering). ";
   typedef std::vector< PCPointerType,
                        Eigen::aligned_allocator< PCPointerType > >
           VectorPCPointerType;
   VectorPCPointerType buildingClouds = 
           SegmentBuildingClouds< PCType >( buildingCloud, true );
    
   std::cerr << "      Number of segment buildings: " 
             << buildingClouds.size() << std::endl;

   std::cerr << "  Segmentation done." << std::endl;

   ////////////////////////////////////////////////////////////////
   // Retrieve individual buildings represented as PolygonMesh:
   std::cerr << "  Starting building mesh reconstruction): ";

   typedef pcl::PolygonMesh::Ptr PolygonPtrType;
   std::vector< PolygonPtrType,
                Eigen::aligned_allocator< PolygonPtrType > > buildingMeshes;
   for( int i = 0; i < buildingClouds.size(); i++ )
   {
      // Reconstruct the mesh out of the extracted building cloud point:
      PolygonPtrType newBuildingMesh = ReconstructBuildingEnveloppe< PCType >(
                                          buildingClouds[ i ],
                                          planeCoefficients );
      std::cerr << newBuildingMesh->polygons.size() << ", ";
      buildingMeshes.push_back( newBuildingMesh );
   }
   std::cerr << " Done. " << std::endl;

   return std::make_tuple( buildingClouds, buildingMeshes );
}

} // namespace testing
} // namespace vcity

#endif // __PCLEXTRACTBUILDINGFROMLIDAR_TXX__
