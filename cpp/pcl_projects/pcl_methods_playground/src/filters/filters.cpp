#include "filters.hpp"

Filters::Filters(){}

void Filters::pass_through(pcl::PointCloud <pcl::PointXYZRGB>::Ptr input_cloud, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (input_cloud);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits (zmin, zmax);
  pass.filter(*filtered_cloud);

  pass.setInputCloud(filtered_cloud);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits (xmin, xmax);
  pass.filter(*filtered_cloud);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits (ymin, ymax);
  pass.filter(*filtered_cloud);

  filtered_cloud_ptr = filtered_cloud;
}

void Filters::medianFilter(pcl::PointCloud <pcl::PointXYZRGB>::Ptr input_cloud, int windowSize)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_smoothed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::MedianFilter<pcl::PointXYZRGB> fbf;
  fbf.setInputCloud(input_cloud);
  //fbf.setMaxAllowedMovement (1.1);
  fbf.setWindowSize (windowSize);
  fbf.applyFilter (*temp_smoothed_cloud);

  smoothed_median_cloud_ptr = temp_smoothed_cloud;
}

void Filters::MLS_smoothing(pcl::PointCloud <pcl::PointXYZRGB>::Ptr input_cloud, float searchRadius)
{
  // The output will also contain the normals.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_smoothed_normal_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_smoothed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Smoothing object (we choose what point types we want as input and output).
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> filter;
	filter.setInputCloud(input_cloud);
	// Use all neighbors in a radius of 3cm.
	filter.setSearchRadius(searchRadius);
	// If true, the surface and normal are approximated using a polynomial estimation
	// (if false, only a tangent one).
	filter.setPolynomialFit(true);
	// We can tell the algorithm to also compute smoothed normals (optional).
	filter.setComputeNormals(true);
	// kd-tree object for performing searches.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;
	filter.setSearchMethod(kdtree);
	filter.process(*temp_smoothed_normal_cloud);
  // Convert from PointXYZRGBNormal to PointXYZRGB
  pcl::copyPointCloud(*temp_smoothed_normal_cloud, *temp_smoothed_cloud);
  smoothed_MLS_normal_cloud_ptr = temp_smoothed_normal_cloud;
  smoothed_MLS_cloud_ptr = temp_smoothed_cloud;
}
