/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/median_filter.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>


class Smoothing {
public:
  Smoothing();

  void medianFilter(pcl::PointCloud <pcl::PointXYZRGBA>::Ptr, int);
  void MLSSmoothing(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC_ptr, float);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smoothed_median_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothed_MLS_normal_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed_MLS_cloud_ptr;

private:

};
