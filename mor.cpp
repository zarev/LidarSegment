
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/pcl_base.h>

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);


  // load the point cloud
  pcl::io::loadPCDFile("points.pcd", *cloud);
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

// Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (5.0f);
  pmf.extract (ground->indices);

  // // Create the filtering object
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud (cloud);
  // extract.setIndices (ground);
  // extract.filter (*cloud_filtered);

  // // Extract non-ground returns
  // extract.setNegative (true);
  // extract.filter (*cloud_filtered);

  // // std::cerr << "Object cloud after filtering: " << std::endl;
  // // std::cerr << *cloud_filtered << std::endl;

  // // pcl::PCDWriter writer;
  // // writer.write<pcl::PointXYZ> ("samp11-utm_ground.pcd", *cloud_filtered, false);

  // // writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);

  // pcl::visualization::CloudViewer viewer("Cloud Viewer");
  // //blocks until the cloud is rendered
  // viewer.showCloud(cloud_filtered);

  // //open viewer
  // while(!viewer.wasStopped()){}


  return (0);
}