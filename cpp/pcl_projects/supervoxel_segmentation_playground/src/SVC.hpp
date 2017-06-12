/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include "../defines.hpp"
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

class SVC {
public:
  SVC();
  void clearOldPolygons(boost::shared_ptr<pcl::visualization::PCLVisualizer> &);
  void drawAdjacencyGraph(boost::shared_ptr<pcl::visualization::PCLVisualizer> &);
  void clearOldSupervoxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> &);
  void showSuperVoxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> &, uint32_t);
  void segmentPC(pcl::PointCloud <PointT>::Ptr, float, float, float, float, float, int);
  void extractSuperVoxelFeatures();
  void fixIds();

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters_fixed;
  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters_refined;

  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency_fixed;

  pcl::PointCloud<PointT>::Ptr rgs_segmented_cloud_ptr;
  pcl::PointCloud<PointT>::Ptr voxel_centroid_cloud;

  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud;
  pcl::PointCloud<pcl::PointXYZL>::Ptr voxel_colored_cloud;
  pcl::PointCloud<pcl::PointXYZL>::Ptr voxel_colored_cloud_refined;

  pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud;
private:
  std::vector<std::string> polyIDs;
  std::vector<std::string> supervoxelIDs;
};
