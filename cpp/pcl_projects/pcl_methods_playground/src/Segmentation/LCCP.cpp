#include "LCCP.hpp"

LCCP::LCCP()
{


}

void LCCP::segmentPC( pcl::PointCloud <pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                      std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr > supervoxel_clusters,
                      std::multimap<uint32_t, uint32_t> supervoxel_adjacency,
                      pcl::PointCloud<pcl::PointXYZL>::Ptr voxel_colored_cloud,
                      float voxel_resolution,
                      float seed_resolution,
                      float concavity_tolerance_threshold,
                      float smoothness_threshold,
                      uint32_t min_segment_size,
                      int use_extended_convexity,
                      bool use_sanity_criterion)
{

  pcl::LCCPSegmentation<pcl::PointXYZRGB> lccp;
  lccp.setConcavityToleranceThreshold (concavity_tolerance_threshold);
  lccp.setSanityCheck (use_sanity_criterion);
  lccp.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
  lccp.setKFactor (use_extended_convexity);
  lccp.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
  lccp.setMinSegmentSize (min_segment_size);
  lccp.segment ();

  PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = voxel_colored_cloud;
  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud_temp_ptr = sv_labeled_cloud->makeShared ();
  lccp.relabelCloud (*lccp_labeled_cloud_temp_ptr);
  pcl::LCCPSegmentation<pcl::PointXYZRGB>::SupervoxelAdjacencyList sv_adjacency_list;
  lccp.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization

  lccp_labeled_cloud_ptr = lccp_labeled_cloud_temp_ptr;

}
