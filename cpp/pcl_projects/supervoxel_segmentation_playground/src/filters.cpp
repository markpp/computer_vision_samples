#include "filters.hpp"

Filters::Filters()
{

}

void Filters::pass_through(pcl::PointCloud <PointT>::Ptr inputPC, pcl::PointCloud <PointT>::Ptr outputPC, float zmin, float zmax, float xmin, float xmax, float ymin, float ymax)
{
  // Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (inputPC);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits (zmin, zmax);
  pass.filter (*outputPC);

  pass.setInputCloud (outputPC);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits (xmin, xmax); //RS 57
  //pass.setFilterLimits (-1.05, 1.05); //kinect 84
  //pass.setFilterLimitsNegative (true);
  pass.filter (*outputPC);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits (ymin, ymax); //RS 57
  //pass.setFilterLimits (-1.2, 1.2); //kinect 84
  //pass.setFilterLimitsNegative (true);
  pass.filter (*outputPC);

}
