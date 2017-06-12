#include "CRGS.hpp"

CRGS::CRGS()
{


}

// Creating color histograms for backprojection
void CRGS::segmentPC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC)
{
  // Color-based region growing segmentation

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (inputPC);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (4);
  reg.setPointColorThreshold (5);
  reg.setRegionColorThreshold (6);
  reg.setMinClusterSize (800);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  crgs_segmented_cloud_ptr = reg.getColoredCloud ();

}
