#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <typeinfo>


int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("cloud_cluster_1.pcd", *cloud);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //blocks until the cloud is rendered
  viewer.showCloud(cloud);

  //open viewer
  while(!viewer.wasStopped()){}

  return (0);

}