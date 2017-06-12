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

class CRGS {
public:
  CRGS();

  void segmentPC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr crgs_segmented_cloud_ptr;


private:

};
