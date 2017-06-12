/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

class CustomRGS {
public:
  CustomRGS();

  void segmentPC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr custom_cloud_ptr;


private:

};
