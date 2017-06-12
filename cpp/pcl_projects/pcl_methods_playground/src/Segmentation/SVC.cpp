#include "SVC.hpp"

SVC::SVC()
{


}

void SVC::segmentPC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC, float voxel_resolution, float seed_resolution, float color_importance, float spatial_importance, float normal_importance, int refinement_itr)
{
  pcl::SupervoxelClustering<pcl::PointXYZRGB> super (voxel_resolution, seed_resolution);
  //if (disable_transform)
  //  super.setUseSingleCameraTransform (false);
  super.setInputCloud (inputPC);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);


  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

  voxel_centroid_cloud = super.getVoxelCentroidCloud ();

  labeled_voxel_cloud = super.getLabeledVoxelCloud ();

  voxel_colored_cloud = super.getLabeledCloud ();

  super.refineSupervoxels(refinement_itr, supervoxel_clusters_refined);

  voxel_colored_cloud_refined = super.getLabeledCloud ();

  sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");


  //pcl::console::print_highlight ("Getting supervoxel adjacency\n");

  super.getSupervoxelAdjacency (supervoxel_adjacency);
}
