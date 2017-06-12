/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// The segmentation class this example is for
#include <pcl/segmentation/lccp_segmentation.h>

class LCCP {
public:
  LCCP();

  void segmentPC( pcl::PointCloud <pcl::PointXYZRGB>::Ptr,
                  std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr >,
                  std::multimap<uint32_t, uint32_t> supervoxel_adjacency,
                  pcl::PointCloud<pcl::PointXYZL>::Ptr,
                  float, float, float, float,
                  uint32_t, int, bool);

  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud_ptr;

private:

};
