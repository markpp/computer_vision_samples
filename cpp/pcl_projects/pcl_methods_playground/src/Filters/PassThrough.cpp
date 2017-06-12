#include "PassThrough.hpp"

PassThrough::PassThrough()
{

}

void PassThrough::filter(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC, pcl::PointCloud <pcl::PointXYZRGB>::Ptr outputPC, float zmin, float zmax, float xmin, float xmax, float ymin, float ymax)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
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
