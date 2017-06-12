/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/ply_io.h>

class Conversion {
public:
  Conversion();
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgba2xyzrgb(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputPC, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPC);
  void appendNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals_ptr;
private:

};
