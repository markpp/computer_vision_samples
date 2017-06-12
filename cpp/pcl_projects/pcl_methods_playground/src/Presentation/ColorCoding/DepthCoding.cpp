#include "DepthCoding.hpp"

DepthCoding::DepthCoding()
{

}

void DepthCoding::colorDepth(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC_ptr)
{

  //pcl::PointCloud<pcl::PointXYZRGB> depthColored_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_depthColored_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  int indexMaxZ = 0;
  int indexMinZ = 0;
  for(int k=0;k<inputPC_ptr->width;k++)
  {
    if(inputPC_ptr->points[indexMaxZ].z < inputPC_ptr->points[k].z)
    {
      indexMaxZ = k;
    }
    if(inputPC_ptr->points[indexMinZ].z > inputPC_ptr->points[k].z)
    {
      indexMinZ = k;
    }
  }
  float valueMaxZ = inputPC_ptr->points[indexMaxZ].z;
  float valueMinZ = inputPC_ptr->points[indexMinZ].z;
  std::cout << " z min: " << valueMinZ << " z max: " << valueMaxZ << std::endl;

  // Fill in the cloud data
  temp_depthColored_cloud_ptr->width    = (int) inputPC_ptr->points.size();
  temp_depthColored_cloud_ptr->height   = 1;
  temp_depthColored_cloud_ptr->is_dense = false;
  temp_depthColored_cloud_ptr->points.resize (temp_depthColored_cloud_ptr->width * temp_depthColored_cloud_ptr->height);

  std::cout << " test: " << std::endl;
  for(int k=0;k<inputPC_ptr->width;k++)
  {
    int scaledValue = 0 + ((255 - 0) / (valueMaxZ - valueMinZ)) * (inputPC_ptr->points[k].z - valueMinZ);

    unsigned char r = colormap[3*scaledValue+2];
    unsigned char g = colormap[3*scaledValue+1];
    unsigned char b = colormap[3*scaledValue];
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
      static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    temp_depthColored_cloud_ptr->points[k].rgb = *reinterpret_cast<float*>(&rgb);
    temp_depthColored_cloud_ptr->points[k].x = inputPC_ptr->points[k].x;
    temp_depthColored_cloud_ptr->points[k].y = inputPC_ptr->points[k].y;
    temp_depthColored_cloud_ptr->points[k].z = inputPC_ptr->points[k].z;
  }
  depthColored_cloud_ptr = temp_depthColored_cloud_ptr;

}
