#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>



int main(int argc, char** argv){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // load the point cloud
  pcl::io::loadPCDFile("points.pcd", *cloud);

  ////ROI////
  // build the condition: restrict in y
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
                  pcl::ConditionAnd<pcl::PointXYZ>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
      pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -25.0)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
      pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 15)));

  // build the roi filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud);
  condrem.setKeepOrganized(true);
  // apply roi filter
  condrem.filter(*cloud_filtered);
  ////ROI////
  
  ////REMOVE GROUND////
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // define the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg_gr;
  seg_gr.setOptimizeCoefficients(true);
  seg_gr.setModelType(pcl::SACMODEL_PLANE);
  seg_gr.setMethodType(pcl::SAC_RANSAC);
  seg_gr.setDistanceThreshold(0.3);
  seg_gr.setInputCloud(cloud_filtered);
  seg_gr.segment(*inliers, *coefficients);

  //remove the ground plane
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  //true leaves non ground points
  extract.setNegative(true);
  extract.filter(*cloud_filtered);
  ////REMOVE GROUND////
  
  //cluster remainder//
  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud_filtered);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (5.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);

  // Extract non-ground returns
  extract.setNegative (true);
  extract.filter (*cloud_filtered);
  

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //blocks until the cloud is rendered
  viewer.showCloud(cloud_filtered);
  //open viewer
  while(!viewer.wasStopped()){}

  return(0);
}