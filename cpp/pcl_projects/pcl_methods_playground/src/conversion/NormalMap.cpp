#include "NormalMap.hpp"

NormalMap::NormalMap()
{
}

void NormalMap::generateNormals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC_ptr)
{
/*
  //pcl::copyPointCloud(*cloud, *normalcloud);
  pcl::PointCloud<pcl::Normal>::Ptr temp_cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (inputPC_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.005);
  ne.compute (*temp_cloud_normals);
  std::cout << "normals calculated" << std::endl;
*/

// The output will also contain the normals.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // Smoothing object (we choose what point types we want as input and output).
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> filter;
	filter.setInputCloud(inputPC_ptr);
	// Use all neighbors in a radius of 3cm.
	filter.setSearchRadius(0.01);
	// If true, the surface and normal are approximated using a polynomial estimation
	// (if false, only a tangent one).
	filter.setPolynomialFit(true);
	// We can tell the algorithm to also compute smoothed normals (optional).
	filter.setComputeNormals(true);
	// kd-tree object for performing searches.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;
	filter.setSearchMethod(kdtree);

	filter.process(*smoothedCloud);
  //pcl::io::savePLYFileASCII("../output/output.ply", *smoothedCloud);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud_normals = smoothedCloud;

  cv::Mat img = cv::Mat::zeros(temp_cloud_normals->height,temp_cloud_normals->width, CV_8UC3);
  ///*
  //for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = inputPC_ptr->begin(); it!= inputPC_ptr->end(); it++)
  //for(unsigned int i=0; i<temp_cloud_normals->size(); i++)
  std::cout << "Cloud height: " << temp_cloud_normals->height << ", cloud width: " << temp_cloud_normals->width << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_normals_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_normals_tmp->points.resize (temp_cloud_normals->width * temp_cloud_normals->height);

  for(unsigned int i=0; i<temp_cloud_normals->size(); i++)
  {

    if(pcl::isFinite(temp_cloud_normals->points[i]))
    {
      //pcl::PointXYZRGB p;
      unsigned char r = (uchar)(std::abs(temp_cloud_normals->points[i].normal_z)*255);
      unsigned char g = (uchar)(std::abs(temp_cloud_normals->points[i].normal_y)*255);
      unsigned char b = (uchar)(std::abs(temp_cloud_normals->points[i].normal_x)*255);
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      cloud_normals_tmp->points[i].rgb = *reinterpret_cast<float*>(&rgb);
      cloud_normals_tmp->points[i].x = temp_cloud_normals->points[i].x;
      cloud_normals_tmp->points[i].y = temp_cloud_normals->points[i].y;
      cloud_normals_tmp->points[i].z = temp_cloud_normals->points[i].z;
    }

  }
  cloud_normals_tmp->width  = temp_cloud_normals->width;
  cloud_normals_tmp->height = temp_cloud_normals->height;
/*
  for(unsigned int v=0; v<temp_cloud_normals->height; v++)
  {
    for(unsigned int u=0; u<temp_cloud_normals->width; u++)
    {
      pcl::PointNormal p = temp_cloud_normals->points[v * temp_cloud_normals->width + u];
      if(pcl::isFinite(p))
      {
        //std::cout << p.normal_x << ", " << p.normal_y << ", " << p.normal_z << std::endl;
        //std::cout << inputPC_ptr->points[i].x << ", " << inputPC_ptr->points[i].y << ", " << inputPC_ptr->points[i].z << std::endl;

        img.at<cv::Vec3b>(v, u)[0] = (uchar)(std::abs(p.normal_x)*255);
        img.at<cv::Vec3b>(v, u)[1] = (uchar)(std::abs(p.normal_y)*255);
        img.at<cv::Vec3b>(v, u)[2] = (uchar)(std::abs(p.normal_z)*255);
        int test = img.at<cv::Vec3b>(v, u)[0];


        //std::cout << test << std::endl;
        //std::cout << img.at<cv::Vec3b>(v, u)[0] << ", " << img.at<cv::Vec3b>(v, u)[1] << ", " << img.at<cv::Vec3b>(v, u)[2] << std::endl;

        //std::cout << it->x << ", " << it->y << ", " << it->z << std::endl;
      }
    }
  }
  */
  //cv::imwrite("../output/mapped.png", img);
  //*/
  std::cout << "Cloud height: " << cloud_normals_tmp->height << ", cloud width: " << cloud_normals_tmp->width << std::endl;

  //pcl::copyPointCloud(*temp_cloud_normals, *cloud_normals);
  //pcl::io::savePLYFileASCII("../output/output.ply", *cloud_normals_tmp);


  cloud_normals = cloud_normals_tmp;
}

/*


for(unsigned int v=0; v<cloud.height(); v++)
{
  for(unsigned int u=0; u<cloud.width(); u++)
  {
    cloud(u,v).x = u;
  }
}
*/
