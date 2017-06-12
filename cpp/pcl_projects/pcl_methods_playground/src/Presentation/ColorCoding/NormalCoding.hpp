/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL_INSTANTIATE(NormalCoding, (pcl::PointXYZINormal) (pcl::PointXYZRGBNormal));
//http://pointclouds.org/documentation/tutorials/writing_new_classes.php
class NormalCoding {
  public:
    NormalCoding();

    void colorNormals(pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr, int);
    //pcl::PointCloud<pcl::PointXYZRGB> depthColored_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normalColored_cloud_ptr;


  private:


};
