/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

class NormalMap {
  public:
    NormalMap();
    void generateNormals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC_ptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_normals;
  private:

};
