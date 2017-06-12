/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


class PassThrough {
public:
  PassThrough();

  void filter(pcl::PointCloud <pcl::PointXYZRGB>::Ptr, pcl::PointCloud <pcl::PointXYZRGB>::Ptr, float, float, float, float, float, float);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_ptr;


private:

};
