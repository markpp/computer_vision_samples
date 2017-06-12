#include "NDT.hpp"

NDT::NDT()
{
}

void NDT::registerCloudPair(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  ///*
  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.004, 0.004, 0.004);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;
  //*/

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.001);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.01);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (0.1);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (135);

  // Setting point cloud to be aligned.
  ndt.setInputSource (input_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
  //Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  //Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (0.0, 0.0, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  transformed_input_cloud_ptr = output_cloud;

  // Saving transformed input cloud.
  //pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  //std::cout << "PointCloud " << cloud_xyzrgb_filtered_ptr->points.size() << " data points after filtering. Height: " << cloud_xyzrgb_filtered_ptr->height << " width: " << cloud_xyzrgb_filtered_ptr->width << std::endl;


}
