#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <string>

boost::shared_ptr<pcl::visualization::PCLVisualizer> init_visualizer()
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->addCoordinateSystem (0.01);
  viewer->initCameraParameters ();
  return (viewer);
}

void pass_through_filter_point_cloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC, pcl::PointCloud <pcl::PointXYZRGB>::Ptr outputPC, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setUserFilterValue(0);
  pass.setKeepOrganized(true); // by default filtered point are now set to nan

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

int main(int argc, char ** argv)
{
  std::string root_path, view_id;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = init_visualizer();
  int counter = 0;
  while (!viewer->wasStopped ())
  {
    // set paths for input and output
    std::string input_pc_path = "cloud_0319.pcd";
    std::string output_pc_path = "cloud_0319_filtered.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_pc_path, *cloud_xyzrgb_ptr) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
    //pcl::io::savePCDFileASCII(input_pc_path, *cloud_xyzrgb_ptr);

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pass_through_filter_point_cloud(cloud_xyzrgb_ptr,
                                    transformed_cloud, 
                                    0.01, 0.5,
                                    -1.5, 1.5,
                                    -0.5, 1.5);

    pcl::io::savePCDFileASCII(output_pc_path, *transformed_cloud);

    break;
    if (!viewer->updatePointCloud (transformed_cloud, "cloud"))
    {
      viewer->addPointCloud(transformed_cloud);
    }
    //pcl::io::savePCDFileBinary(output_pc_path, *transformed_cloud);
    
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud seg");

    viewer->spinOnce (1);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));

  }
  return 0;
}
