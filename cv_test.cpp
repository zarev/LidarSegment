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

void viewerSetup(pcl::visualization::PCLVisualizer& viewer){

    viewer.setBackgroundColor(0.4, 0.4, 0.4);

}
int 
main (int argc, char** argv)
{
  // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile("points.pcd", *cloud);
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 3.0);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-25, 17);
  pass.filter (*indices);

  pcl::MinCutSegmentation<pcl::PointXYZ> seg;
  seg.setInputCloud (cloud);
  seg.setIndices (indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointXYZ point(0,0,0);
  // point.x = 0;
  // point.y = 0;
  // point.z = 0;
  foreground_points->points.push_back(point);
  seg.setForegroundPoints (foreground_points);
  seg.setSigma (0.25);
  seg.setRadius (3.0433856);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (1);

  std::vector <pcl::PointIndices> clusters;
  seg.extract(clusters);
  
  // for(auto i:clusters){std::cout<< i << endl;}

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();

  // //return only non-ground points
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud (cloud);
  // extract.setIndices (ground);
  // //extract.filter (*cloud_filtered);
  // extract.setNegative (true);
  // extract.filter (*colored_cloud);
  // extract.setInputCloud(cloud);
  // extract.setIndices(clusters);
  // // extract.setNegative(true);
  // extract.filter(*colored_cloud);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //blocks until the cloud is rendered
  viewer.showCloud(colored_cloud);

  while(!viewer.wasStopped()){}
  

  return (0);
}
