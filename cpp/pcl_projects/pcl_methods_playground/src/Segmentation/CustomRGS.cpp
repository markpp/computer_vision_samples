#include "CustomRGS.hpp"

CustomRGS::CustomRGS()
{

  //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  //pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);

}

// not done
bool CustomRGS::customRegionGrowing (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (squared_distance < 10000)
  {
    if (std::fabs (point_a.r - point_b.r ) < 8.0f)
      return (true);
    if (std::fabs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  }
  else
  {
    if (std::fabs (point_a.r - point_b.r) < 3.0f)
      return (true);
  }
  return (false);
}

// Creating color histograms for backprojection
void CustomRGS::segmentPC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC)
{
  pcl::copyPointCloud (*cloud_xyzrgb_filtered_ptr, *cloud_custom);
  // Set up a Normal Estimation class and merge data in cloud_with_normals
  std::cerr << "Computing normals...\n", tt.tic ();
  pcl::copyPointCloud (*cloud_custom, *cloud_with_normals);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (cloud_custom);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (300.0);
  ne.compute (*cloud_with_normals);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  // Set up a Conditional Euclidean Clustering class
  std::cerr << "Segmenting to clusters...\n", tt.tic ();
  pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
  cec.setInputCloud (cloud_with_normals);
  cec.setConditionFunction (&customRegionGrowing);
  cec.setClusterTolerance (500.0);
  cec.setMinClusterSize (cloud_with_normals->points.size () / 1000);
  cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);
  cec.segment (*clusters);
  cec.getRemovedClusters (small_clusters, large_clusters);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

}
