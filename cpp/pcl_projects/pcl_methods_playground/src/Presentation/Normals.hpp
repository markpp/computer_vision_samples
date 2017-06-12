/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>


class Normals {
public:
  Normals();

  void generateNormals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr, float);
  //void generateCloudNormals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr);
  //
  pcl::PointCloud<pcl::Normal>::Ptr normals_ptr;
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr;
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;


private:


};
