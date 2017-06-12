#include "RGS.hpp"

RGS::RGS()
{


}

void RGS::segmentPC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC, pcl::PointCloud <pcl::Normal>::Ptr normals, int numberOfNeighbours, float smoothnessThreshold, float curvatureThreshold)
{
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

/*
// alternative normals estimation
  pcl::PointCloud <pcl::Normal>::Ptr normal (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (inputPC);
  normal_estimator.setKSearch (30);
  //normal_estimator.setRadiusSearch (0.005);
  normal_estimator.compute (*normal);
*/

  //std::cout << "normals: " << normals->size () << std::endl;
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (250);
  reg.setMaxClusterSize (10000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (numberOfNeighbours);
  reg.setInputCloud (inputPC);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (smoothnessThreshold / 180.0 * M_PI); // If the deviation between points normals is less than smoothness threshold then they are suggested to be in the same cluster
  reg.setCurvatureThreshold (curvatureThreshold); //If two points have a small normals deviation then the disparity between their curvatures is tested. And if this value is less than curvature threshold then the algorithm will continue the growth of the cluster using new added point.

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;

  rgs_segmented_cloud_ptr = reg.getColoredCloud ();
  //cloud_normals = normals;
}
