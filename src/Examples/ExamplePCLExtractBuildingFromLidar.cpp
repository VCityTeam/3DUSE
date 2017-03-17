#include <iostream>
#include <tuple>
#include <vector>
#include <boost/filesystem.hpp>

#include <pcl/console/parse.h>        // For pcl::console
#include <pcl/point_types.h>          // For pcl::PointXYZ  ...
#include <pcl/surface/gp3.h>          // For pcl::PolygonMesh
#ifdef __clang__
  // In order to silence out warnings about VTK deprecated code invocation
  // made by pcl_visualizer.hpp:
  #define VTK_LEGACY_SILENT
  // We also want to silence out vtk complains about member function not being
  // marked override:
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Winconsistent-missing-override"
  #include <pcl/visualization/cloud_viewer.h>
  #pragma clang diagnostic pop
#else
  #include <pcl/visualization/cloud_viewer.h>
#endif

#include "PCLExtractBuildingFromLidar.tpp"

/**
 * @brief A simplification oriented (hidding away some details) wrapping class
 *        for pcl::visualization::PCLVisualizer and that only allows for
 *        trivial display of point clouds and Polygon meshes.
 *        Usage:
 *         - use h key for help (in invocation console)
 *         - use f key to set up a point of the cloud as space origin (for
 *           rotation) but first make sure to select the point (left click)
 *           in the cloud.
 * @sa    Refer to
 *        <a href="http://pointclouds.org/documentation/tutorials/pcl_visualizer.php">PCL's visualizer tutorial</a>
 *        for original tutorial code
 */
template< typename TPointType >
class SimplePclVisualizer {
public:
   typedef TPointType                             PointType;
   typedef typename  pcl::PointCloud< PointType > PCType;
   typedef pcl::PolygonMesh                       MeshType;
   typedef typename PCType::Ptr                   PCPointerType;
   typedef typename PCType::ConstPtr              PCConstPointerType;
   typedef boost::shared_ptr< pcl::visualization::PCLVisualizer >
                                                  VisualizerSharedType;
   
   SimplePclVisualizer();
   void VisualizerAddCloud(   PCConstPointerType cloud, 
                              std::string cloudName );
   void VisualizerAddPolygon( MeshType    mesh,
                              std::string meshName );
   void Run();

private:
   VisualizerSharedType _visualizer;
   int numPolygonMesh = 0;
};

template< typename TPointType >
SimplePclVisualizer< TPointType >:: SimplePclVisualizer()
{
   _visualizer = VisualizerSharedType(
                   new pcl::visualization::PCLVisualizer( "3D Viewer" ) );
   _visualizer->setBackgroundColor( 0, 0, 0 );
   _visualizer->addCoordinateSystem( 1.0 );
   _visualizer->initCameraParameters();
}

template< typename TPointType >
void SimplePclVisualizer< TPointType >::Run()
{
   while ( !_visualizer->wasStopped() )
   {
      _visualizer->spinOnce( 100 );
      boost::this_thread::sleep( boost::posix_time::microseconds( 100000 ) );
   }
}

template< typename TPointType >
void
SimplePclVisualizer< TPointType >::VisualizerAddCloud(
                                        PCConstPointerType cloud,
                                        std::string cloudName )
{
   _visualizer->addPointCloud< PointType >( cloud, cloudName );
   _visualizer->setPointCloudRenderingProperties( 
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName );
}

template< typename TPointType >
void
SimplePclVisualizer< TPointType >::VisualizerAddPolygon(
                                        MeshType    mesh,
                                        std::string meshName )
{
   /// Colors used 
   const std::vector< std::vector< float > > colors
        { { 1.0, 0.0, 0.0  }
        , { 0.0, 1.0, 0.0  }
        , { 0.0, 0.0, 1.0  }
        , { 0.7, 0.7, 0.7  }
        , { 0.7, 0.0, 0.0  }
        , { 0.0, 0.7, 0.0  }
        , { 0.0, 0.0, 0.7  }
        , { 0.4, 0.0, 0.0  }
        , { 0.0, 0.4, 0.0  }
        , { 0.0, 0.0, 0.4  }
        } ;
   _visualizer->addPolygonMesh( mesh, meshName, 0 );
   int ci = (numPolygonMesh++) % 10;         /// Color Index
   _visualizer->setPointCloudRenderingProperties( 
      pcl::visualization::PCL_VISUALIZER_COLOR, 
      colors[ci][0],
      colors[ci][1],
      colors[ci][2],
       meshName );
}

//////////////////////////////////////////////////////////////////
void usage( int narg, char** argv )
{
   std::cerr << "  Wrong number of arguments. "
             << std::endl
             << "  Usage: " << argv[0]
             << "  lidar_data_filename citygml_filename" 
             << std::endl
             << "  where the lidar data is the input information."
             << std::endl
             << "  Note: a small sized lidar data example file can be found VCity sources at"
             << std::endl
             << "        src/filters/Test/PCL/Data/Grand_Lyon_Lidar_1843_5176_sub_sample.las ."
             << std::endl
             << "  Interaction within the display window:"
             << std::endl
             << "   - left click (hold) + mouse for rotation,"
             << std::endl
             << "   - right click (hold) + mouse for zoom in/out,"
             << std::endl
             << "   - hit q key to quit visualization window,,"
             << std::endl
             << "   - hit h key for help (in invocation console)"
             << std::endl
             << "   - select a point of the cloud (with left click) then "
                "hit f key in order to"
             << std::endl
             << "     set that point as new origin for rotations."
             << std::endl
             << "Exiting."
             << std::endl;
   exit( EXIT_FAILURE );
}

/////////////////////////
int main(int narg, char *argv[])
{
   std::cerr << "  Entering example " << argv[0] << std::endl;

   ///// Command line arguments sanity check
   if (   narg < 2 
       || pcl::console::find_argument( narg, argv, "-h")    >= 0
       || pcl::console::find_argument( narg, argv, "-help") >= 0 )
   {
      usage( narg, argv );
   }

   namespace fs = boost::filesystem;
   fs::path   LiDAR_Path( argv[1] );
   std::cerr << "  Using file " <<   LiDAR_Path << " as input."  << std::endl;

   ///// PCL related types and variables
   typedef pcl::PointXYZ                PointType;
   typedef pcl::PointCloud< PointType > PCType;
   typedef PCType::Ptr                  PCPointerType;
   
   std::vector< PCPointerType,
                Eigen::aligned_allocator< PCPointerType > > buildingClouds;
                
   typedef pcl::PolygonMesh::Ptr PolygonPtrType;
   std::vector< PolygonPtrType,
                Eigen::aligned_allocator< PolygonPtrType > > buildingMeshes;
                
   ///// Proper (PCL) building extraction
   namespace vc = vcity::testing;
   std::tie( buildingClouds, buildingMeshes ) =
             vc::ExtractBuildingsPointCloudsAndMeshes< PCType >( LiDAR_Path );
 
   SimplePclVisualizer< PointType > viewer;
   for( int i = 0; i < buildingClouds.size(); i++ )
   {
      viewer.VisualizerAddCloud( buildingClouds[i],
                                  "cloud" + std::to_string( i ) );
   }
   for( int i = 0; i < buildingMeshes.size(); i++ )
   {
      viewer.VisualizerAddPolygon( *buildingMeshes[i],
                                  "mesh" + std::to_string( i ) );
   }
   viewer.Run();

   return 0;
}
