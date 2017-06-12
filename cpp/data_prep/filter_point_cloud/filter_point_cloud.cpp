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

void pass_through_filter_point_cloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC, pcl::PointCloud <pcl::PointXYZRGB>::Ptr outputPC, float zmin, float zmax, float xmin, float xmax, float ymin, float ymax)
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

int main(int argc, char ** argv)
{
  std::string root_path, view_id;
  root_path = argv[1];
  view_id = argv[2];
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = init_visualizer();
  int counter = 0;
  while (!viewer->wasStopped ())
  {
    // set paths for input and output
    std::string input_pc_path = root_path + "/COLOR_PC/COLOR_PC_BIN_" + std::to_string(counter) + view_id + ".pcd";
    std::string output_pc_path = root_path + "/FILTERED_COLOR_PC/FILTERED_COLOR_PC_BIN_" + std::to_string(counter) + view_id + ".pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_pc_path, *cloud_xyzrgb_ptr) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    if(view_id.compare("_C_A") == 0 || view_id.compare("_R_A") == 0)
    {
      pass_through_filter_point_cloud(cloud_xyzrgb_ptr ,transformed_cloud, 0.0, 0.37, -0.1, 0.1, -0.1, 0.04);
    }
    else
    {
      pass_through_filter_point_cloud(cloud_xyzrgb_ptr ,transformed_cloud, 0.29, 0.37, -0.1, 0.1, -0.1, 0.04);
    }


    if (!viewer->updatePointCloud (transformed_cloud, "cloud"))
    {
      viewer->addPointCloud(transformed_cloud);
    }
    pcl::io::savePCDFileBinary(output_pc_path, *transformed_cloud);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud seg");

    viewer->spinOnce (1);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    counter++;
    // check if final cloud has been reached
    if(counter > 150) // 500
    {
      // exit if final cloud was reached
      break;
      //reset counter to run through all PC again
      //counter = 0; //9
    }
  }
  return 0;
}
