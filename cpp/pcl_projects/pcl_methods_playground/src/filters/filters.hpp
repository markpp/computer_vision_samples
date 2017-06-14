/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/median_filter.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>

#include "../../defines.hpp"

class Filters {
public:
  Filters();
  void pass_through(pcl::PointCloud <pcl::PointXYZRGB>::Ptr, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);
  void medianFilter(pcl::PointCloud <pcl::PointXYZRGB>::Ptr, int);
  void MLS_smoothing(pcl::PointCloud <pcl::PointXYZRGB>::Ptr, float);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed_median_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothed_MLS_normal_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed_MLS_cloud_ptr;
private:

};
