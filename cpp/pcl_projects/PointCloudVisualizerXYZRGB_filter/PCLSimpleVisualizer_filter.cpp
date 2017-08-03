#include <boost/thread/thread.hpp>
#include <string>

#include "src/filters.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

//typedef PointCloud<PointT> CloudT;

boost::shared_ptr<pcl::visualization::PCLVisualizer> initVisualizer ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->setBackgroundColor (255, 255, 255);

  viewer->addCoordinateSystem (0.05, -0.05, -0.06, 0.30);
  viewer->initCameraParameters ();
  // adding sphere, useful for seeing what the viewer i centering on
  pcl::PointXYZ FocusPoint;
  FocusPoint.x=0.0; FocusPoint.y=-0.1; FocusPoint.z=0.2; //FocusPoint.z=0.2;

  pcl::PointXYZ SpherePoint;
  SpherePoint.x=-0.1; SpherePoint.y=-0.1; SpherePoint.z=0.42; //FocusPoint.z=0.2;

  //viewer->addSphere (SpherePoint,0.01,"sp");
  viewer->setCameraPosition(200.0, 80.0, -400.0, 0.0, 0.0, FocusPoint.z, 0.05, 1.0, 0.0); // cam in z = -0.2, focus on z = 0.2
  return (viewer);
}

int main ( int argc, char** argv )
{
  // Filter object for preprocessing point cloud
  Filters filters;

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  viewer = initVisualizer();
  std::string path = argv[1];
  //std::string path = "/Users/markpp/Desktop/data/kinect2/IV1cm57/frame_9_20151205T200624.169120.pcd";
  if (pcl::io::loadPCDFile<PointT> (path, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  //PCL_INFO("The cloud is %s.\n", cloud->is_dense ? "dense" : "NOT dense");
  // write .ply PC, readable by Meshlab, meshlab cannot read PC files which contain NaN
  //pcl::io::savePLYFileASCII("../output/PLYxyzRGBcloud.ply",*cloud);

  std::vector<pcl::visualization::Camera> cam;
  //Save the position of the camera
  viewer->getCameras(cam);
  //Print recorded points on the screen:
  cout << "Cam: " << endl
       << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl
       << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl
       << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
  pcl::PointCloud<PointT>::Ptr cloud_workingCopy (new pcl::PointCloud<PointT>);

/*
  // ct back
  float xmin = 50.0;
  float xmax = 400.0;
  float ymin = 140.0;
  float ymax = 240.0;
  float zmin = -50.0;
  float zmax = 700.0;
*/


  // ct midt
  float xmin = -50.0;
  float xmax = 500.0;
  float ymin = 100.0;
  float ymax = 240.0;
  float zmin = -50.0;
  float zmax = 800.0;

/*
  //kinect midt
  float xmin = -0.25;
  float xmax = 0.45;
  float ymin = -0.42;
  float ymax = 0.21;
  float zmin = 0.6;
  float zmax = 0.8;
*/

  /*
  filters.pass_through(cloud, cloud_workingCopy, zmin, zmax, xmin, xmax, ymin, ymax);

  boost::replace_all(path, "ply", "pcd");
  cout << path << endl;
  pcl::io::savePCDFileASCII(path, *cloud_workingCopy);
*/
  while (!viewer->wasStopped ())
  {

    if (!viewer->updatePointCloud(cloud, "cloud"))
    {
      viewer->addPointCloud(cloud, "cloud");
    }
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->spinOnce (1);
    //viewer->saveScreenshot("../output/screenshot.png");
    //while(1){}
  }
  return 0;
}
