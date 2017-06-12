#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <string>

boost::shared_ptr<pcl::visualization::PCLVisualizer> initVisualizer ()
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

/*
void PassThroughFilter(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC, pcl::PointCloud <pcl::PointXYZRGB>::Ptr outputPC, float zmin, float zmax, float xmin, float xmax, float ymin, float ymax)
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
*/
void rotatePC(pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputPC, pcl::PointCloud <pcl::PointXYZRGB>::Ptr outputPC)
{
	/* 
      Reminder: how transformation matrices work :

      |-------> This column is the translation
      | 1 0 0 x |  \
      | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
      | 0 0 1 z |  /
      | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

      METHOD #1: Using a Matrix4f
      This is the "manual" method, perfect to understand but error prone !
    
     Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = M_PI/2; // The angle of rotation in radians
    transform_1 (0,0) = cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = cos (theta);
    //    (row, column)
    */
   
  // Print the transformation
    printf ("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    //  METHOD #2: Using a Affine3f
    //This method is easier and less error prone
    
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; tetha radians arround Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl; 

   // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);
}

int main ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = initVisualizer();
  int counter = 0;
  while (!viewer->wasStopped ())
  {
    // set paths for input and output
    std::string path = "/Users/markpp/Desktop/code/VAPprojects/PCLRotatePCs/build/out/PC_BIN_" + std::to_string(counter) + "_FRONT.pcd";
    std::string outputString = "../output/PC_BIN_" + std::to_string(counter) + "_FRONT.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *cloud) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
    
    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    
    rotatePC(cloud ,transformed_cloud);
    if (!viewer->updatePointCloud (transformed_cloud, "cloud"))
    {
      viewer->addPointCloud(transformed_cloud);
    }
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud seg");

    viewer->spinOnce (1);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    counter++;
    // check if final cloud has been reached
    if(counter == 145) // 500
    {
      // exit if final cloud was reached
      break;
      //reset counter to run through all PC again
      //counter = 0; //9
    }
  }
  return 0;
}
