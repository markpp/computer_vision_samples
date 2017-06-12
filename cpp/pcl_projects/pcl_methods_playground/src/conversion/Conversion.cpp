#include "Conversion.hpp"

Conversion::Conversion()
{
}

void Conversion::appendNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPC_ptr, pcl::PointCloud<pcl::Normal>::Ptr input_Normals_ptr)
{

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  //pcl::concatenateFields(inputPC_ptr, input_Normals_ptr, output_cloud_tmp);

  output_cloud_tmp->points.resize (inputPC_ptr->width * inputPC_ptr->height);

  std::cout << "Combining XYZRGB and Normals: " << "XYZRGB size: "  << inputPC_ptr->size()  << "Normals size: " << input_Normals_ptr->size() << std::endl;

  for(unsigned int i=0; i<inputPC_ptr->size(); i++)
  {
    //if(pcl::isFinite(inputPC_ptr->points[i]))
    //{
      //pcl::PointXYZRGB p;
      output_cloud_tmp->points[i].x = inputPC_ptr->points[i].x;
      output_cloud_tmp->points[i].y = inputPC_ptr->points[i].y;
      output_cloud_tmp->points[i].z = inputPC_ptr->points[i].z;
      output_cloud_tmp->points[i].rgb = inputPC_ptr->points[i].rgb;
      output_cloud_tmp->points[i].normal_x = input_Normals_ptr->points[i].normal_x;
      output_cloud_tmp->points[i].normal_y = input_Normals_ptr->points[i].normal_y;
      output_cloud_tmp->points[i].normal_z = input_Normals_ptr->points[i].normal_z;
      output_cloud_tmp->points[i].curvature = input_Normals_ptr->points[i].curvature;
    //}
  }
  output_cloud_tmp->width  = inputPC_ptr->width;
  output_cloud_tmp->height = inputPC_ptr->height;

  cloud_normals_ptr = output_cloud_tmp;
}

//Convert from PointXYZRGBA to PointXYZRGB
//pcl::copyPointCloud(*cloud_xyzrgba_ptr, *cloud_xyzrgb);
//Convert from PointXYZRGB to PointXYZ
//pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
