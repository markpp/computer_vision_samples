/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>

class ICP {
public:
  ICP();
  void registerCloudPair(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_input_cloud_ptr;
private:
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

};
