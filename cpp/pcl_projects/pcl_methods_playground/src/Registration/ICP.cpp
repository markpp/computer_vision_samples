#include "ICP.hpp"

ICP::ICP()
{
}

void ICP::registerCloudPair(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
  // The Iterative Closest Point algorithm
  //.tic ();
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setMaximumIterations (250);
  icp.setInputSource (input_cloud);
  icp.setInputTarget (target_cloud);
  icp.align (*input_cloud);
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  //std::cout << "Applied " << iterations << " ICP iteration(s) in " << std::endl;

  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    //std::cout << "\nICP transformation " << iterations << " : input_cloud -> input_cloud" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    //print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    //return (-1);
  }
  //// Executing the transformation
  //pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
  //transformed_input_cloud_ptr = output_cloud;

  // Saving transformed input cloud.
  //pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  //std::cout << "PointCloud " << cloud_xyzrgb_filtered_ptr->points.size() << " data points after filtering. Height: " << cloud_xyzrgb_filtered_ptr->height << " width: " << cloud_xyzrgb_filtered_ptr->width << std::endl;


}
