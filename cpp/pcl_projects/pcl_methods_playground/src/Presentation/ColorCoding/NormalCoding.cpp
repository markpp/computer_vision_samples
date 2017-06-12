#include "NormalCoding.hpp"

NormalCoding::NormalCoding()
{

}

void NormalCoding::colorNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud_with_normals, int coding)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
  output_cloud_tmp->points.resize (input_cloud_with_normals->width * input_cloud_with_normals->height);

  for(unsigned int i=0; i<input_cloud_with_normals->size(); i++)
  {

    if(pcl::isFinite(input_cloud_with_normals->points[i]))
    {
      // z should probably be fliped to have same direction as the eye/cam
      unsigned char b, g, r;
      if(coding == 1)//STANDARD
      {
        //conversion from normal vector to RGB normal map: color = (component + 1)/2
        //conversion from RGB normal map to normal vector: component = color*2 - 1
        b = (uchar)((((input_cloud_with_normals->points[i].normal_z*-1)+1)/2)*255);
        g = (uchar)(((input_cloud_with_normals->points[i].normal_y+1)/2)*255);
        r = (uchar)(((input_cloud_with_normals->points[i].normal_x+1)/2)*255);
      }
      else //ABSOLUTE
      {
        b = (uchar)(std::abs(input_cloud_with_normals->points[i].normal_z)*255);
        g = (uchar)(std::abs(input_cloud_with_normals->points[i].normal_y)*255);
        r = (uchar)(std::abs(input_cloud_with_normals->points[i].normal_x)*255);
      }

      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      output_cloud_tmp->points[i].rgb = *reinterpret_cast<float*>(&rgb);
      output_cloud_tmp->points[i].x = input_cloud_with_normals->points[i].x;
      output_cloud_tmp->points[i].y = input_cloud_with_normals->points[i].y;
      output_cloud_tmp->points[i].z = input_cloud_with_normals->points[i].z;
    }

  }
  output_cloud_tmp->width  = input_cloud_with_normals->width;
  output_cloud_tmp->height = input_cloud_with_normals->height;

  normalColored_cloud_ptr = output_cloud_tmp;
}
