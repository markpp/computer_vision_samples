/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

class RGS {
public:
  RGS();

  void segmentPC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr, pcl::PointCloud <pcl::Normal>::Ptr, int, float, float);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgs_segmented_cloud_ptr;
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;


private:

};
