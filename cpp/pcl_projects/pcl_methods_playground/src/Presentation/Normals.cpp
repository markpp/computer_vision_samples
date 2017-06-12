#include "Normals.hpp"

Normals::Normals()
{

}

void Normals::generateNormals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC_ptr, float radius)
{

  //pcl::copyPointCloud(*cloud, *normalcloud);
  pcl::PointCloud<pcl::Normal>::Ptr temp_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (inputPC_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (radius);
  ne.compute (*temp_normals_ptr);
  std::cout << "normals calculated" << std::endl;

  normals_ptr = temp_normals_ptr;
}
/*
void Normals::generateCloudNormals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC_ptr)
{

  //pcl::copyPointCloud(*cloud, *normalcloud);

  pcl::PointCloud<pcl::Normal>::Ptr temp_cloud_normals_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (inputPC_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.005);
  ne.compute (*temp_cloud_normals_ptr);
  std::cout << "normals calculated" << std::endl;

  cloud_normals_ptr = temp_cloud_normals_ptr;
}
*/
