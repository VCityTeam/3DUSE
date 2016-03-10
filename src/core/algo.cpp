// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "algo.hpp"
////////////////////////////////////////////////////////////////////////////////
#if 1
#include "application.hpp"
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>
#include "gui/moc/mainWindow.hpp"
#include "export/exportCityGML.hpp"
#include "src/processes/ExportToShape.hpp"
#include "src/gui/osg/osgGDAL.hpp"
#include "citymodel.hpp"
////////////////////////////////////////////////////////////////////////////////
#include <lasreader.hpp>
#include <laswriter.hpp>

#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/visualization/cloud_viewer.h>

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

////////////////////////////////////////////////////////////////////////////////

namespace vcity
{
	Algo::Algo()
	{
	}
	////////////////////////////////////////////////////////////////////////////////
	Algo::~Algo()
	{
	}
	////////////////////////////////////////////////////////////////////////////////
	/**
	* @brief ProjectPointOnPolygon3D : prend un point 2D Point et calcule sa coordonnée Z en partant du principe qu'il est coplanaire à Polygon
	* @param Point : point que l'on veut extruder en 3D
	* @param Polygon : polygon qui définit le plan sur lequel vient se poser Point
	* @return le point 3D correspondant
	*/
	OGRPoint* ProjectPointOnPolygon3D(OGRPoint* Point, OGRPolygon* Polygon)
	{
		OGRLinearRing* Ring = Polygon->getExteriorRing();

		TVec3d A;
		TVec3d B;
		TVec3d C;

		A.x = Ring->getX(0);
		A.y = Ring->getY(0);
		A.z = Ring->getZ(0);

		TVec3d AB;
		TVec3d AC;

		int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
		for(int i = 1; i < Ring->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une répétition du premier
		{
			if(test == 0)
			{
				B.x = Ring->getX(i);
				B.y = Ring->getY(i);
				B.z = Ring->getZ(i);

				if(A.x != B.x || A.y != B.y)
				{
					++test;// A est bien différent de B
					AB = B - A;
				}
			}
			else if(test == 1)
			{
				C.x = Ring->getX(i);
				C.y = Ring->getY(i);
				C.z = Ring->getZ(i);

				if((C.x - A.x)/(B.x - A.x) != (C.y - A.y)/(B.y - A.y))
				{
					++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
					AC = C - A;
					break;
				}
			}
		}

		if(test != 2)
		{
			std::cout << "Erreur lors de la creation du plan. \n";
			return nullptr;
		}

		TVec3d M; // <=> Point
		M.x = Point->getX();
		M.y = Point->getY();

		double s, t;

		t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
		s = (M.x - A.x - t * AC.x) / AB.x;

		M.z = A.z + s * AB.z + t * AC.z;

		return new OGRPoint(M.x, M.y, M.z);
	}

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ExtractPlansFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud2)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>(*Cloud2));

		pcl::PCDWriter writer;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Res;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (1000);
		seg.setDistanceThreshold (0.5);

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		int i = 0, nr_points = (int) Cloud->points.size ();
		// While 30% of the original cloud is still there
		while (Cloud->points.size () > 0.1 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (Cloud);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the inliers
			extract.setInputCloud (Cloud);
			extract.setIndices (inliers);
			extract.setNegative (false);
			extract.filter (*cloud_p);
			std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

			std::stringstream ss;
			ss << "Plane_" << i << ".pcd";
			writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

			Res.push_back(cloud_p);

			// Create the filtering object
			extract.setNegative (true);
			extract.filter (*cloud_f);
			Cloud.swap (cloud_f);
			i++;
		}

		return Res;
	}

	void SegmentationByNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud2)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(*Cloud2));

		///The smallest scale to use in the DoN filter.
		double scale1 = 0.2;//0.5

		///The largest scale to use in the DoN filter.
		double scale2 = 2;//1

		///The minimum DoN magnitude to threshold by
		double threshold = 0.25; //0.1
		//std::cout << "threshold ? " << std::endl;
		//std::cin >> threshold;

		///segment scene into clusters with given distance tolerance using euclidean clustering
		double segradius = 0.2;//3
		//std::cout << "segradius ? " << std::endl;
		//std::cin >> segradius;

		// Create a search tree, use KDTreee for non-organized data.
		pcl::search::Search<pcl::PointXYZ>::Ptr tree;
		tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));

		// Set the input pointcloud for the search tree
		tree->setInputCloud (cloud);

		// Compute normals using both small and large scales at each point
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
		ne.setInputCloud (cloud);
		ne.setSearchMethod (tree);

		/**
		* NOTE: setting viewpoint is very important, so that we can ensure
		* normals are all pointed in the same direction!
		*/
		ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

		// calculate normals with the small scale
		std::cout << "Calculating normals for scale..." << scale1 << std::endl;
		pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

		ne.setRadiusSearch (scale1);
		ne.compute (*normals_small_scale);

		// calculate normals with the large scale
		std::cout << "Calculating normals for scale..." << scale2 << std::endl;
		pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

		ne.setRadiusSearch (scale2);
		ne.compute (*normals_large_scale);

		// Create output cloud for DoN results
		pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud, *doncloud);

		std::cout << "Calculating DoN... " << std::endl;
		// Create DoN operator
		pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
		don.setInputCloud (cloud);
		don.setNormalScaleLarge (normals_large_scale);
		don.setNormalScaleSmall (normals_small_scale);

		if (!don.initCompute ())
		{
			std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
			exit (EXIT_FAILURE);
		}

		// Compute DoN
		don.computeFeature (*doncloud);

		std::cout << "Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

		pcl::PCDWriter writer;
		// Save DoN features
		doncloud->width = (int)doncloud->points.size();
		doncloud->height = 1;

		writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

		// Filter by magnitude
		std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

		// Build the condition for filtering
		pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
			new pcl::ConditionOr<pcl::PointNormal> ()
			);
		range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
			new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
			);
		// Build the filter
		pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
		condrem.setInputCloud (doncloud);

		pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

		// Apply filter
		condrem.filter (*doncloud_filtered);

		doncloud = doncloud_filtered;

		// Save filtered output
		std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

		doncloud->width = (int)doncloud->points.size();
		doncloud->height = 1;
		writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

		// Filter by magnitude
		std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;

		pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
		segtree->setInputCloud (doncloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

		ec.setClusterTolerance (segradius);
		ec.setMinClusterSize (50);//50
		ec.setMaxClusterSize (100000);
		ec.setSearchMethod (segtree);
		ec.setInputCloud (doncloud);
		ec.extract (cluster_indices);

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<pcl::PointNormal>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				cloud_cluster_don->points.push_back (doncloud->points[*pit]);
			}

			cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
			cloud_cluster_don->height = 1;
			cloud_cluster_don->is_dense = true;

			//Save cluster
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
			std::stringstream ss;
			ss << "don_cluster_" << j << ".pcd";
			cloud_cluster_don->width = (int)cloud_cluster_don->points.size();
			cloud_cluster_don->height = 1;
			writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
			++j;
		}
	}

	void SavePCDtoLAS(std::string FileName, pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud)
	{
		LASwriteOpener laswriteopener;
		laswriteopener.set_file_name(FileName.c_str());

		// init header

		LASheader lasheader;
		lasheader.x_scale_factor = 1;
		lasheader.y_scale_factor = 1;
		lasheader.z_scale_factor = 1;
		lasheader.x_offset = appGui().getSettings().getDataProfile().m_offset.x;
		lasheader.y_offset = appGui().getSettings().getDataProfile().m_offset.y;
		lasheader.z_offset = 0.0;
		lasheader.point_data_format = 1;
		lasheader.point_data_record_length = 28;

		// init point 

		LASpoint laspoint;
		laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

		// open laswriter

		LASwriter* laswriter = laswriteopener.open(&lasheader);

		// write points

		for (pcl::PointXYZ Point : Cloud->points)
		{
			// populate the point

			laspoint.set_X(Point.x);
			laspoint.set_Y(Point.y);
			laspoint.set_Z(Point.z);

			// write the point

			laswriter->write_point(&laspoint);

			// add it to the inventory

			laswriter->update_inventory(&laspoint);
		}

		// update the header

		laswriter->update_header(&lasheader, TRUE);

		// close the writer

		I64 total_bytes = laswriter->close();

		delete laswriter;

		std::cout << "Fichiers " << FileName << " cree" << std::endl;
	}

	////////////////////////////////////////////////////////////////////////////////

	/**
	* @brief Extrait les zones composées de nombreux points proches
	* @param cloud : nuage de points en entrée
	* @return un nuage de point avec les zones colorées selon les regroupements
	*/
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentationRoofs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		OGRMultiPoint* Points = new OGRMultiPoint; //liste des points en sortie pour export Shapefile

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointsSegmentes; //Liste des groupements de points

		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (1);
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (2500000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud);
		ec.extract (cluster_indices);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_trie (new pcl::PointCloud<pcl::PointXYZRGB>);

		int j = 0;

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			double r = 255 * rand() / (RAND_MAX + 1.0f);
			double g = 255 * rand() / (RAND_MAX + 1.0f);
			double b = 255 * rand() / (RAND_MAX + 1.0f);
			//std::cout << r << " " << g << " " << b << std::endl;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				cloud_cluster->points.push_back (cloud->points[*pit]); //*
			cloud_cluster->width = (int)cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			PointsSegmentes.push_back(cloud_cluster);

			pcl::PointXYZRGB PointRGB;

			for(pcl::PointXYZ P : cloud_cluster->points)
			{
				Points->addGeometry(new OGRPoint(P.x, P.y, P.z));

				PointRGB.x = P.x;
				PointRGB.y = P.y;
				PointRGB.z = P.z;
				PointRGB.r = r;
				PointRGB.g = g;
				PointRGB.b = b;

				Cloud_trie->points.push_back(PointRGB);
			}

			//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			//std::stringstream ss;
			//ss << "1845_5181_cloud_cluster_" << j << ".pcd";
			//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
			//j++;
		}

		Cloud_trie->width = (int)Cloud_trie->points.size ();
		Cloud_trie->height = 1;

		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZRGB>("ChangementsSegmente.pcd", *Cloud_trie, false);

		SaveGeometrytoShape("ChangementsSegmenteShp.shp", Points);

		delete Points;

		return PointsSegmentes;
	}

	/**
	* @brief Extrait les points d'un nuage de point situé à plus d'une certaine distance d'un autre nuage
	* @param Cloud1 : Nuage de points de référence
	* @param Cloud2 : Nuage de points que l'on va comparer à Cloud1
	* @param Distance : Distance minimale à partir de laquelle un point de Cloud2 sera jugé comme suffisament éloigné de Cloud1, et mis de côté
	* @return Un nuage de points contenant tous les points de Cloud2 situés à plus de Distance de Cloud1
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractDistantPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud2, float Distance)
	{
		pcl::PointXYZ point;

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(Cloud1);

		int K = 1; //K plus proches voisins

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudColore (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr CloudModif (new pcl::PointCloud<pcl::PointXYZ>);

		for(size_t i = 0; i < Cloud2->size(); ++i)
		{
			point = Cloud2->at(i);

			pcl::PointXYZRGB P;
			pcl::PointXYZ P2;

			std::vector<int> IndicesPoints(K);
			std::vector<float> Distances(K);

			if( kdtree.nearestKSearch(point, K, IndicesPoints, Distances) > 0 )
			{
				P.x = point.x;
				P2.x = point.x;
				P.y = point.y;
				P2.y = point.y;
				P.z = point.z;
				P2.z = point.z;

				P.r = Distances[0]*25.5; //Rouge à 10m de différence.
				P.g = Distances[0]*25.5;
				P.b = Distances[0]*25.5;

				CloudColore->points.push_back(P);

				if(Distances[0] > Distance)
				{
					CloudModif->points.push_back(P2);
				}
			}
		}

		CloudColore->width = (int)CloudColore->points.size();
		CloudColore->height = 1;
		CloudColore->is_dense = false;

		CloudModif->width = (int)CloudModif->points.size();
		CloudModif->height = 1;
		CloudModif->is_dense = false;

		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZRGB> ("ComparaisonLidar.pcd", *CloudColore, false);

		writer.write<pcl::PointXYZ> ("Changements.pcd", *CloudModif, false);

		return CloudModif;
	}

	pcl::PointCloud<pcl::Normal>::Ptr ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

		ne.setKSearch(4);
		//ne.setRadiusSearch (5); 

		// Compute the features
		ne.compute (*cloud_normals);

		return cloud_normals;
	}
	////////////////////////////////////////////////////////////////////////////////
	/**
	* @brief Détecte les changements sur les bâtiments entre deux fichiers LiDAR. Le plus récent est en 2.
	* @param Path1 : Chemin pour le fichier correspondant au fichier LiDAR le plus ancien
	* @param Path2 : Chemin pour le fichier correspondant au fichier LiDAR le plus récent
	* @return 
	*/
	void Algo::CompareTwoLidar(std::string Path1, std::string Path2)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCDReader reader;
		//reader.read<pcl::PointXYZ> ("C://Users///FredLiris//Downloads//PCL//table_scene_lms400.pcd", *Cloud);
		reader.read<pcl::PointXYZ> ("C://VCity.git//VCity Build//ChangementsSegmente.pcd", *Cloud);

		OGRMultiLineString* NormalesTest = new OGRMultiLineString;

		pcl::PointCloud<pcl::Normal>::Ptr NormalesCloud = ComputeNormals(Cloud);

		SavePCDtoLAS("test.las", Cloud);

		for(size_t i = 0; i < Cloud->points.size(); ++i)
		{
			OGRLineString* Line = new OGRLineString;
			Line->addPoint(Cloud->points.at(i).x, Cloud->points.at(i).y, Cloud->points.at(i).z);
			Line->addPoint(Cloud->points.at(i).x + NormalesCloud->points.at(i).normal_x, Cloud->points.at(i).y + NormalesCloud->points.at(i).normal_y, Cloud->points.at(i).z + NormalesCloud->points.at(i).normal_z);

			NormalesTest->addGeometryDirectly(Line);
		}

		SaveGeometrytoShape("NormalesTest.shp", NormalesTest);
		delete NormalesTest;

		return;

		LASreadOpener lasreadopener;
		lasreadopener.set_file_name(Path1.c_str());
		LASreader* LiDAR1 = lasreadopener.open();
		lasreadopener.set_file_name(Path2.c_str());
		LASreader* LiDAR2 = lasreadopener.open();

		pcl::PointXYZ point;
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

		while (LiDAR1->read_point())
		{
			point.x = (LiDAR1->point).get_x() - appGui().getSettings().getDataProfile().m_offset.x;
			point.y = (LiDAR1->point).get_y() - appGui().getSettings().getDataProfile().m_offset.y;
			point.z = (LiDAR1->point).get_z();

			Cloud1->points.push_back(point);
		}

		Cloud1->width = (int)Cloud1->points.size();
		Cloud1->height = 1;
		Cloud1->is_dense = false;

		//pcl::PCDWriter writer;
		//writer.write<pcl::PointXYZ> ("LiDAR2012.pcd", *Cloud1, false);

		while (LiDAR2->read_point())
		{
			point.x = (LiDAR2->point).get_x() - appGui().getSettings().getDataProfile().m_offset.x;
			point.y = (LiDAR2->point).get_y() - appGui().getSettings().getDataProfile().m_offset.y;
			point.z = (LiDAR2->point).get_z();

			Cloud2->points.push_back(point);
		}

		Cloud2->width = (int)Cloud2->points.size();
		Cloud2->height = 1;
		Cloud2->is_dense = false;

		//writer.write<pcl::PointXYZ> ("LiDAR2015.pcd", *Cloud2, false);

		std::cout << "Ouverture des LiDAR terminee." << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr CloudModif = ExtractDistantPoints(Cloud1, Cloud2, 2.0f);

		std::cout << "Extraction Points distances terminee." << std::endl;

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointsSegmentes = SegmentationRoofs(CloudModif);

		std::cout << "Comparaison terminee." << std::endl;

		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr Test (new pcl::PointCloud<pcl::PointXYZRGB>);

		//pcl::PointXYZRGB pointRGB;

		OGRMultiLineString* Normales = new OGRMultiLineString;

		for(pcl::PointCloud<pcl::PointXYZ>::Ptr PointsCloud : PointsSegmentes)
		{
			pcl::PointCloud<pcl::Normal>::Ptr NormalsCloud = ComputeNormals(PointsCloud);

			//std::cout << PointsCloud->points.size() << " " << NormalsCloud->points.size() << std::endl;

			for(size_t i = 0; i < PointsCloud->points.size(); ++i)
			{
				//pointRGB.x = PointsCloud->points.at(i).x;
				//pointRGB.y = PointsCloud->points.at(i).y;
				//pointRGB.z = PointsCloud->points.at(i).z;
				//pointRGB.r = 128 * (1+NormalsCloud->points.at(i).normal_x);
				//pointRGB.g = 128 * (1+NormalsCloud->points.at(i).normal_y);
				//pointRGB.b = 128 * (1+NormalsCloud->points.at(i).normal_z);

				//std::cout << NormalsCloud->points.at(i).normal_x << " " << NormalsCloud->points.at(i).normal_y << " " << NormalsCloud->points.at(i).normal_z << std::endl;

				//std::cout << unsigned(pointRGB.r) << " " << unsigned(pointRGB.g) << " " << unsigned(pointRGB.b) << std::endl;
				//int a;
				//std::cin >> a;

				//Test->points.push_back(pointRGB);

				OGRLineString* Line = new OGRLineString;
				Line->addPoint(PointsCloud->points.at(i).x, PointsCloud->points.at(i).y, PointsCloud->points.at(i).z);
				Line->addPoint(PointsCloud->points.at(i).x + NormalsCloud->points.at(i).normal_x, PointsCloud->points.at(i).y + NormalsCloud->points.at(i).normal_y, PointsCloud->points.at(i).z + NormalsCloud->points.at(i).normal_z);

				Normales->addGeometryDirectly(Line);
			}
		}

		SaveGeometrytoShape("Normales.shp", Normales);
		delete Normales;

		//Test->width = (int)Test->points.size();
		//Test->height = 1;
		//Test->is_dense = false;

		//pcl::PCDWriter writer;
		//writer.write<pcl::PointXYZRGB> ("Normals.pcd", *Test, false);

		std::cout << "Calcul des normales termine." << std::endl;
	}


	////////////////////////////////////////////////////////////////////////////////
	void Algo::ConvertLasToPCD()
	{
		LASreadOpener lasreadopener;
		//lasreadopener.set_file_name("C://Users//FredLiris//Downloads//1845_5181.las");
		lasreadopener.set_file_name("C://Users//FredLiris//Downloads//Grand Lyon LiDAR//Grand Lyon CHANGEMENTS CRAPONNE//LAS 2015//1833_5173_2015.las");
		LASreader* lasreader = lasreadopener.open();

		pcl::PointXYZ point;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		while (lasreader->read_point())
		{
			point.x = (lasreader->point).get_x();
			point.y = (lasreader->point).get_y();
			point.z = (lasreader->point).get_z();

			cloud->points.push_back(point);
		}

		cloud->width = (int)cloud->points.size();
		cloud->height = 1;
		cloud->is_dense = false;

		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("1845_5181.pcd", *cloud, false);

		std::cout << "Fichier PCD cree." << std::endl;
	}

	void Algo::ExtractGround()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndicesPtr ground (new pcl::PointIndices);

		// Fill in the cloud data
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZ> ("C://Users//FredLiris//Downloads//1845_5181.pcd", *cloud);

		std::cerr << "Cloud before filtering: " << std::endl;
		std::cerr << *cloud << std::endl;

		// Create the filtering object
		pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
		pmf.setInputCloud (cloud);
		pmf.setMaxWindowSize (20);
		pmf.setSlope (1.0f);
		pmf.setInitialDistance (0.5f);
		pmf.setMaxDistance (3.0f);
		pmf.extract (ground->indices);

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (ground);
		extract.filter (*cloud_filtered);

		std::cerr << "Ground cloud after filtering: " << std::endl;
		std::cerr << *cloud_filtered << std::endl;

		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("1845_5181_ground.pcd", *cloud_filtered, false);

		// Extract non-ground returns
		extract.setNegative (true);
		extract.filter (*cloud_filtered);

		std::cerr << "Object cloud after filtering: " << std::endl;
		std::cerr << *cloud_filtered << std::endl;

		writer.write<pcl::PointXYZ> ("1845_5181_object.pcd", *cloud_filtered, false);
	}

	void Algo::ExtractBuildings()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>), cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);;

		// Fill in the cloud data
		pcl::PCDReader reader;
		pcl::PCDWriter writer;

		reader.read<pcl::PointXYZ> ("C://Users//FredLiris//Downloads//1845_5181_NoGround_2m.pcd", *cloud);

		std::cerr << "PointCloud : " << cloud->width * cloud->height << " data points." << std::endl;

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cloud_trie = SegmentationRoofs(cloud);
	}

	void Algo::PrepareTIN()
	{
		std::string TINPath = "D:/Donnees/Data/Donnees_Sathonay/SATHONAY_CAMP_MNT_2012.gml";
		std::cout << "load citygml file : " << TINPath << std::endl;
		vcity::Tile* tile = new vcity::Tile(TINPath);

		citygml::CityModel* model = tile->getCityModel();

		OGRMultiPolygon* TinTerrain = new OGRMultiPolygon;

		for(citygml::CityObject* obj : model->getCityObjectsRoots())
		{
			if(obj->getType() == citygml::COT_TINRelief)
			{
				for(citygml::Geometry* Geometry : obj->getGeometries())
				{
					for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
					{
						OGRPolygon * OgrPoly = new OGRPolygon;
						OGRLinearRing * OgrRing = new OGRLinearRing;

						for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
						{
							OgrRing->addPoint(Point.x, Point.y, Point.z);
						}

						OgrRing->closeRings();

						if(OgrRing->getNumPoints() > 3)
						{
							OgrPoly->addRingDirectly(OgrRing);
							if(OgrPoly->IsValid())
								TinTerrain->addGeometryDirectly(OgrPoly);
						}
						else
							delete OgrRing;
					}
				}
			}
		}

		delete tile;
		SaveGeometrytoShape("TIN.shp", TinTerrain);

		OGRGeometry* Envelope = new OGRPolygon;

		for(int i = 0; i < TinTerrain->getNumGeometries(); ++i)
		{
			OGRGeometry* tmp = Envelope;
			Envelope = tmp->Union(TinTerrain->getGeometryRef(i));
			delete tmp;
		}

		SaveGeometrytoShape("TIN_Envelope.shp", Envelope);

		delete TinTerrain;
		delete Envelope;
	}

	void Algo::RemoveGroundWithTIN()
	{		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZ> ("C:/Users/FredLiris/Downloads/1845_5181.pcd", *cloud);
		std::cerr << "Cloud : " << std::endl;
		std::cerr << *cloud << std::endl;

		OGREnvelope* CloudBBox = new OGREnvelope;

		CloudBBox->MinX = cloud->at(0).x;
		CloudBBox->MaxX = cloud->at(0).x;
		CloudBBox->MinY = cloud->at(0).y;
		CloudBBox->MaxY = cloud->at(0).y;

		for(pcl::PointXYZ P : cloud->points)
		{
			if(P.x > CloudBBox->MaxX)
				CloudBBox->MaxX = P.x;
			if(P.x < CloudBBox->MinX)
				CloudBBox->MinX = P.x;
			if(P.y > CloudBBox->MaxY)
				CloudBBox->MaxY = P.y;
			if(P.y < CloudBBox->MinY)
				CloudBBox->MinY = P.y;
		}

		OGRLinearRing * Ring = new OGRLinearRing;
		Ring->addPoint(CloudBBox->MinX, CloudBBox->MinY);
		Ring->addPoint(CloudBBox->MinX, CloudBBox->MaxY);
		Ring->addPoint(CloudBBox->MaxX, CloudBBox->MaxY);
		Ring->addPoint(CloudBBox->MaxX, CloudBBox->MinY);
		Ring->addPoint(CloudBBox->MinX, CloudBBox->MinY);
		OGRPolygon* PolyBBox = new OGRPolygon;
		PolyBBox->addRingDirectly(Ring);
		delete CloudBBox;

		SaveGeometrytoShape("BoundingBox_1845_5181.shp", PolyBBox);

		OGRMultiPolygon* TinTerrain = new OGRMultiPolygon;
		OGRMultiPolygon* TinTerrainEnvelope = new OGRMultiPolygon;

		OGRDataSource* DS_TinTerrain = OGRSFDriverRegistrar::Open("TIN.shp", TRUE);
		OGRDataSource* DS_TinTerrainEnvelope = OGRSFDriverRegistrar::Open("TIN_Envelope.shp", TRUE);
		OGRLayer* L_TinTerrain = DS_TinTerrain->GetLayer(0);
		OGRLayer* L_TinTerrainEnvelope = DS_TinTerrainEnvelope->GetLayer(0);

		OGRFeature* Feature;
		L_TinTerrain->ResetReading();

		while((Feature = L_TinTerrain->GetNextFeature()) != NULL)
		{
			if(Feature->GetGeometryRef()->Intersects(PolyBBox))
				TinTerrain->addGeometry(Feature->GetGeometryRef());
		}

		L_TinTerrainEnvelope->ResetReading();

		while((Feature = L_TinTerrainEnvelope->GetNextFeature()) != NULL)
		{
			if(Feature->GetGeometryRef()->Intersects(PolyBBox))
				TinTerrainEnvelope->addGeometry(Feature->GetGeometryRef()->Intersection(PolyBBox));
		}

		std::cout << "TinTerrain et TinTerrainEnvelope charges." << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudGround (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoGround (new pcl::PointCloud<pcl::PointXYZ>);

		int cpt = 0;

		for(pcl::PointXYZ P : cloud->points)
		{
			std::cout << "Points traites : " << cpt << " / " <<  cloud->points.size() << std::endl;
			++cpt;
			OGRPoint* Point = new OGRPoint(P.x, P.y, P.z);
			if(!Point->Intersects(TinTerrainEnvelope))
				continue;
			for(int i = 0; i < TinTerrain->getNumGeometries(); ++i)
			{
				OGRPolygon* Poly = (OGRPolygon*)TinTerrain->getGeometryRef(i);
				if(!Point->Intersects(Poly))
					continue;

				OGRPoint* PointPoly = ProjectPointOnPolygon3D(Point, Poly);
				if(abs(P.z - PointPoly->getZ()) < 2)
					cloudGround->points.push_back(P);
				else
					cloudNoGround->points.push_back(P);
			}
		}

		cloudGround->width = (int)cloudGround->points.size();
		cloudGround->height = 1;
		cloudGround->is_dense = false;
		cloudNoGround->width = (int)cloudNoGround->points.size();
		cloudNoGround->height = 1;
		cloudNoGround->is_dense = false;

		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("1845_5181_Ground.pcd", *cloudGround, false);
		writer.write<pcl::PointXYZ> ("1845_5181_NoGround.pcd", *cloudNoGround, false);

		std::cout << "LiDAR separe en Ground et NoGround." << std::endl;
	}

	void Algo::CompareBuildings()
	{
		//Ouverture de la Bounding Box de la zone

		OGRDataSource* DS_BBox = OGRSFDriverRegistrar::Open("BoundingBox_1845_5181.shp");
		OGRLayer* L_BBox = DS_BBox->GetLayer(0);
		OGRFeature* Feature;
		L_BBox->ResetReading();
		OGRPolygon* BBox;
		while((Feature = L_BBox->GetNextFeature()) != NULL)
			BBox = (OGRPolygon*)Feature->GetGeometryRef()->clone();
		delete DS_BBox;

		//Ouverture du fichier LiDAR décomposé en ensemble de points voisins pouvant représenter des bâtiments

		pcl::PCDReader reader;
		pcl::PCDWriter writer;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		reader.read<pcl::PointXYZRGB> ("C:/Users/FredLiris/Downloads/1845_5181_NoGround_Trie_1_200.pcd", *cloud);

		std::vector<OGRMultiPoint*> ListCloud;
		OGRMultiPoint* PointsCloud = new OGRMultiPoint;
		double old_r = -1, old_g = -1, old_b = -1;

		OGRGeometryCollection* CloudPolygons = new OGRMultiPolygon;
		OGRGeometryCollection* CloudPolygons2 = new OGRMultiPolygon;

		int cpt = 0;

		for(pcl::PointXYZRGB Point : cloud->points)
		{
			if(Point.r != old_r || Point.g != old_g || Point.b != old_b)
			{
				std::cout << "Avancement : " << cpt << " / " << cloud->points.size() << std::endl;
				if(PointsCloud->getNumGeometries() > 0)
				{
					OGRGeometry* Test1 = PointsCloud->Buffer(2);
					OGRGeometry* Test2 = Test1->Buffer(-2);
					std::cout << Test2->getGeometryName() << std::endl; 
					if(Test2->getGeometryType() == wkbMultiPolygon || Test2->getGeometryType() == wkbMultiPolygon25D)//MultiPolygon à traiter : l'érosion peut couper en deux des polygones
					{
						OGRMultiPolygon* MP = (OGRMultiPolygon*)Test2;
						for(int p = 0; p < MP->getNumGeometries(); ++p)
							CloudPolygons->addGeometry(MP->getGeometryRef(p));
					}
					else
						CloudPolygons->addGeometry(Test2);

					CloudPolygons2->addGeometry(Test1);

					delete Test1;
					delete Test2;

					ListCloud.push_back((OGRMultiPoint*)PointsCloud->clone());
					delete PointsCloud;
					PointsCloud = new OGRMultiPoint;
				}
				old_r = Point.r;
				old_g = Point.g;
				old_b = Point.b;
			}
			PointsCloud->addGeometry(new OGRPoint(Point.x, Point.y, Point.z));
			++cpt;
		}

		SaveGeometrytoShape("CloudPolygons.shp", CloudPolygons);
		SaveGeometrytoShape("CloudPolygons2.shp", CloudPolygons2);

		OGRDataSource* DS = OGRSFDriverRegistrar::Open("CloudPolygons.shp");
		OGRLayer* Layer = DS->GetLayer(0);
		Layer->ResetReading();
		while((Feature = Layer->GetNextFeature()) != NULL)
			CloudPolygons->addGeometry(Feature->GetGeometryRef());

		//Ouverture des emprises au sol représentant les bâtiments connus

		OGRDataSource* DS_Batis = OGRSFDriverRegistrar::Open("BatisNew.shp");
		OGRLayer* L_Batis = DS_Batis->GetLayer(0);
		L_Batis->ResetReading();
		OGRMultiPolygon* ListBatis = new OGRMultiPolygon;

		while((Feature = L_Batis->GetNextFeature()) != NULL)
		{
			OGRPolygon* Bati = (OGRPolygon*)Feature->GetGeometryRef();

			if(!Bati->Intersects(BBox))
				continue;
			ListBatis->addGeometry(Bati);
		}	
		delete DS_Batis;

		//Comparaison des deux jeux de données

		OGRGeometryCollection* CloudPolygons3 = new OGRMultiPolygon;

		for(int i = 0; i < CloudPolygons->getNumGeometries(); ++i)
		{
			bool exist = false;
			for(int j = 0; j < ListBatis->getNumGeometries(); ++j)
			{
				if(CloudPolygons->getGeometryRef(i)->Intersects(ListBatis->getGeometryRef(j)))
				{
					exist = true;
					break;
				}
			}
			if(exist)
				CloudPolygons3->addGeometry(CloudPolygons->getGeometryRef(i));
		}

		SaveGeometrytoShape("CloudPolygons3.shp", CloudPolygons3);
	}

	void Algo::ConstructRoofs()
	{
		pcl::PCDReader reader;
		pcl::PCDWriter writer;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		reader.read<pcl::PointXYZRGB> ("C:/Users/FredLiris/Downloads/PCL/1845_5181_NoGround_Trie_1_200.pcd", *cloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr currcloud (new pcl::PointCloud<pcl::PointXYZ>);

		double old_r = -1, old_g = -1, old_b = -1;

		int cpt = 0;

		for(pcl::PointXYZRGB Point : cloud->points)
		{
			if(Point.r != old_r || Point.g != old_g || Point.b != old_b)
			{
				std::cout << "Avancement : " << cpt << " / " << cloud->points.size() << std::endl;
				if(currcloud->points.size() > 0)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud2(new pcl::PointCloud<pcl::PointXYZ>(*currcloud));
					Cloud2->width = (int)currcloud->points.size();
					Cloud2->height = 1;
					Cloud2->is_dense = false;
					writer.write<pcl::PointXYZ> ("CurrCloud.pcd", *Cloud2, false);
					//std::cout << "CurrCloud.pcd cree." << std::endl;

					SegmentationByNormal(currcloud);
					//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Planes = ExtractPlansFromCloud(currcloud);
					currcloud->clear();
					std::cout << "Continue ? " << std::endl;
					int a;
					std::cin >> a;
				}
				old_r = Point.r;
				old_g = Point.g;
				old_b = Point.b;
			}
			//std::cout << "Test1" << std::endl;
			currcloud->points.push_back(pcl::PointXYZ(Point.x, Point.y, Point.z));
			//std::cout << "Test2" << std::endl;
			++cpt;
		}
	}
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////

#endif

