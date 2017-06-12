#include "CPC.hpp"

CPC::CPC()
{


}

void CPC::segmentPC( pcl::PointCloud <pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                      std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr > supervoxel_clusters,
                      std::multimap<uint32_t, uint32_t> supervoxel_adjacency,
                      pcl::PointCloud<pcl::PointXYZL>::Ptr voxel_colored_cloud,
                      float voxel_resolution,
                      float seed_resolution,
                      float concavity_tolerance_threshold, //LCCP
                      float smoothness_threshold,
                      uint32_t min_segment_size,
                      int use_extended_convexity,
                      bool use_sanity_criterion,
                      float min_cut_score, // CPC
                      unsigned int max_cuts,
                      unsigned int cutting_min_segments,
                      bool use_local_constrain,
                      bool use_directed_cutting,
                      bool use_clean_cutting,
                      unsigned int ransac_iterations)
{

  PCL_INFO ("Starting Segmentation\n");
  pcl::CPCSegmentation<pcl::PointXYZRGB> cpc;
  cpc.setConcavityToleranceThreshold (concavity_tolerance_threshold);
  cpc.setSanityCheck (use_sanity_criterion);
  cpc.setCutting (max_cuts, cutting_min_segments, min_cut_score, use_local_constrain, use_directed_cutting, use_clean_cutting);
  cpc.setRANSACIterations (ransac_iterations);
  cpc.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
  cpc.setKFactor (use_extended_convexity);
  cpc.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
  cpc.setMinSegmentSize (min_segment_size);
  cpc.segment ();

  PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = voxel_colored_cloud;
  pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud_temp_ptr = sv_labeled_cloud->makeShared ();
  cpc.relabelCloud (*cpc_labeled_cloud_temp_ptr);
  pcl::LCCPSegmentation<pcl::PointXYZRGB>::SupervoxelAdjacencyList sv_adjacency_list;
  cpc.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization

  cpc_labeled_cloud_ptr = cpc_labeled_cloud_temp_ptr;

}
