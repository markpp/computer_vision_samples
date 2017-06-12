/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

class SVC {
public:
  SVC();



  void segmentPC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr, float, float, float, float, float, int);

  std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr > supervoxel_clusters;
  std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr > supervoxel_clusters_refined;

  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgs_segmented_cloud_ptr;
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_centroid_cloud;

  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud;

  pcl::PointCloud<pcl::PointXYZL>::Ptr voxel_colored_cloud;

  pcl::PointCloud<pcl::PointXYZL>::Ptr voxel_colored_cloud_refined;

  pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud;
private:

};
