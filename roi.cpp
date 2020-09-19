//Euclidean Distance Clustering for Point Cloud Segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>

using namespace std;

int
main(int argc, char** argv)
{
	// Objects that declare storage point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//Read Pcd files
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("points.pcd", *cloud) == -1)	
	{		
		PCL_ERROR("Cloudn't read file!");		
		return -1;	
	} cout << "there are " << cloud->points.size()<<" points before filtering." << endl; 	


	// Create kd-tree objects for searching.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);

	// Euclidean clustering objects.
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;//Class Euclidean Cluster Extraction is a class based on Euclidean distance for clustering and segmentation.
	clustering.setClusterTolerance(4.0);//Setting too small search radius in Euclidean space may lead to clustering divided into several clusters, and setting too large may lead to clustering interconnection.
	clustering.setMinClusterSize(100);// Set the minimum number of points contained in the cluster
	clustering.setMaxClusterSize(25000); //Set the maximum number of points contained in the cluster
	clustering.setSearchMethod(kdtree);//Key member functions of classes
	clustering.setInputCloud(cloud);//Clustering and Segmentation of Point Clouds with Specified Input
	std::vector<pcl::PointIndices> clusters;// cluster stores the results of clustering segmentation of point clouds. Point Indices store the index of the corresponding set of points
	clustering.extract(clusters);

	// For every cluster...
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		//Add all point clouds to a new point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// Preservation
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "C://Users//HEHE//Desktop//cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

		currentClusterNum++;
	}
}