#include "Smoothing.hpp"

Smoothing::Smoothing()
{

}

void Smoothing::medianFilter(pcl::PointCloud <pcl::PointXYZRGBA>::Ptr inputPC, int windowSize)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_smoothed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::MedianFilter<pcl::PointXYZRGBA> fbf;
  fbf.setInputCloud(inputPC);
  //fbf.setMaxAllowedMovement (1.1);
  fbf.setWindowSize (windowSize);
  fbf.applyFilter (*temp_smoothed_cloud_ptr);

  smoothed_median_cloud_ptr = temp_smoothed_cloud_ptr;
}

void Smoothing::MLSSmoothing(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC_ptr, float searchRadius)
{

// The output will also contain the normals.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_smoothed_normal_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_smoothed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Smoothing object (we choose what point types we want as input and output).
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> filter;
	filter.setInputCloud(inputPC_ptr);
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

	filter.process(*temp_smoothed_normal_cloud_ptr);

  //pcl::io::savePLYFileASCII("../output/out.ply", *temp_smoothed_normal_cloud_ptr);

  smoothed_MLS_normal_cloud_ptr = temp_smoothed_normal_cloud_ptr;
  //Convert from PointXYZRGBA to PointXYZRGB

  pcl::copyPointCloud(*smoothed_MLS_normal_cloud_ptr, *temp_smoothed_cloud_ptr);

  smoothed_MLS_cloud_ptr = temp_smoothed_cloud_ptr;
}
