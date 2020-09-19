#include <iostream>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>


// 1 https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html?highlight=PassThrough
// 2 https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html#planar-segmentation
// 3 https://pcl.readthedocs.io/projects/tutorials/en/latest/min_cut_segmentation.html?highlight=MinCutSegmentation
// remove ground, filter out points, segment based on distance. 
int main (int argc, char** argv)
{
  pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
  pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud <pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("points.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 3.0);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-25, 17);
  // pass.setNegative(true);
  pass.filter (*cloud_filtered);


  ///REMOVE GROUND////
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_gr (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg_gr;
  seg_gr.setOptimizeCoefficients (true);
  seg_gr.setModelType (pcl::SACMODEL_PLANE);
  seg_gr.setMethodType (pcl::SAC_RANSAC);
  seg_gr.setDistanceThreshold (0.3);

  seg_gr.setInputCloud (cloud_filtered);
  seg_gr.segment (*inliers_gr, *coefficients);

  if (inliers_gr->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  //remove the ground plane
  pcl::ExtractIndices<pcl::PointXYZ> extract_gr;
  extract_gr.setInputCloud(cloud_filtered);
  extract_gr.setIndices(inliers_gr);
  //true leaves non ground points
  extract_gr.setNegative(true);
  extract_gr.filter(*cloud_filtered);
  ////REMOVE GROUND////

  pcl::IndicesPtr indices_obj (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass_obj;
  pass_obj.setInputCloud (cloud_filtered);
  pass_obj.setFilterFieldName ("z");
  pass_obj.setFilterLimits (0.0, 4.0);
  pass_obj.setFilterFieldName ("y");
  pass_obj.setFilterLimits (-20.0, 17.0);
  pass_obj.setNegative(false);
  pass_obj.filter (*indices_obj);

  pcl::MinCutSegmentation<pcl::PointXYZ> seg_obj;
  seg_obj.setInputCloud (cloud_filtered);
  seg_obj.setIndices (indices_obj);

  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointXYZ point;
  point.x = 68.97;
  point.y = -18.55;
  point.z = 0.57;
  foreground_points->points.push_back(point);
  seg_obj.setForegroundPoints (foreground_points);

  seg_obj.setSigma (0.20);
  seg_obj.setRadius (5.0433856);
  seg_obj.setNumberOfNeighbours (14);
  seg_obj.setSourceWeight (0.8);

  std::vector <pcl::PointIndices> clusters;
  seg_obj.extract (clusters);


  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg_obj.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ()){}

  return (0);
}