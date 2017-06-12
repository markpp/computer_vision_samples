/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "../defines.hpp"

class Filters {
public:
  Filters();

  void pass_through(pcl::PointCloud <PointT>::Ptr, pcl::PointCloud <PointT>::Ptr, float, float, float, float, float, float);
  pcl::PointCloud<PointT>::Ptr filtered_cloud_ptr;


private:

};
