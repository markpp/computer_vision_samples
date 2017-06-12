/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <boost/thread/thread.hpp>

class NDT {
public:
  NDT();
  void registerCloudPair(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_input_cloud_ptr;
private:

};
