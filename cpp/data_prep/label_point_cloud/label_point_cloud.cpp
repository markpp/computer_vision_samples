#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <stdint.h>

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

void changeColor(cv::Mat annoMat, pcl::PointCloud <pcl::PointXYZRGBA>::Ptr inputPC, pcl::PointCloud <pcl::PointXYZL>::Ptr outputPC)//, pcl::PointCloud <pcl::PointXYZRGBA>::Ptr outputPCColor)
{
  for(uint64_t i=0;i<inputPC->height;i++)
  {
    for(uint64_t j=0;j<inputPC->width;j++)
    {
      uint64_t cloudIndex = inputPC->width*i+j;
      uint8_t label = annoMat.at<uchar>(i,j);
      if(label > 0)
      {
        pcl::PointXYZL temp_point;
        temp_point.x = inputPC->points[cloudIndex].x;
        temp_point.y = inputPC->points[cloudIndex].y;
        temp_point.z = inputPC->points[cloudIndex].z;
        temp_point.label = label/50-1;
        outputPC->points.push_back(temp_point);
      }
      /*
      if(label > 0 && pcl_isfinite(outputPC->points[cloudIndex].x) && pcl_isfinite(outputPC->points[cloudIndex].y) && pcl_isfinite(outputPC->points[cloudIndex].z))
      {
        //annoMat.at<uchar>(i,j) = (uchar)label;
        outputPC->points[cloudIndex].label = label;
        if(label==100)
        {
          outputPCColor->points[cloudIndex].r = 255;
          outputPCColor->points[cloudIndex].g = 0;
          outputPCColor->points[cloudIndex].b = 0;
        }
        else if(label == 150)
        {
          outputPCColor->points[cloudIndex].r = 0;
          outputPCColor->points[cloudIndex].g = 0;
          outputPCColor->points[cloudIndex].b = 255;
        }
        else if(label == 200)
        {
          outputPCColor->points[cloudIndex].r = 200;
          outputPCColor->points[cloudIndex].g = 0;
          outputPCColor->points[cloudIndex].b = 100;
        }
      }
      else
      {
        outputPC->points[cloudIndex].label = 50;
        //outputPCColor->points[cloudIndex].r = 0;
        //outputPCColor->points[cloudIndex].g = 255;
        //outputPCColor->points[cloudIndex].b = 0;
      }
      */
    }
  }
  outputPC->width = (int) outputPC->points.size();
  outputPC->height = 1;
  outputPC->is_dense = false;
}

int main(int argc, char ** argv)
{
  std::string root_path, view_id;
  root_path = argv[1];
  view_id = argv[2];

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = initVisualizer();

  int counter = 0;
  while (!viewer->wasStopped ())
  {
    // Load input point cloud
    std::string input_pc_path = root_path + "/COLOR_PC/COLOR_PC_BIN_" + std::to_string(counter) + view_id + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (input_pc_path, *cloud_xyzrgba_ptr) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read input pc file xxx.pcd \n");
      return (-1);
    }
    // Load annotation image
    cv::Mat border_label_img = cv::imread(root_path + "/BORDER_ANNO/BORDER_ANNO_" + std::to_string(counter) + view_id + ".png", 0);
    // Rotate image such that indexing matches with point cloud
    transpose(border_label_img,border_label_img);
    flip(border_label_img,border_label_img,1);
    //cv::imshow("border_label_img", border_label_img);
    std::string border_labelled_pc_anno_path = root_path + "/BORDER_LABELLED_PC/BORDER_LABELLED_PC_ASCII_" + std::to_string(counter) + view_id + ".pcd";
    pcl::PointCloud<pcl::PointXYZL>::Ptr border_cloud (new pcl::PointCloud<pcl::PointXYZL> ());
    changeColor(border_label_img, cloud_xyzrgba_ptr, border_cloud);
    pcl::io::savePCDFileASCII(border_labelled_pc_anno_path, *border_cloud);

    // Load annotation image
    cv::Mat complete_label_img = cv::imread(root_path + "/COMPLETE_ANNO/COMPLETE_ANNO_" + std::to_string(counter) + view_id + ".png", 0);
    // Rotate image such that indexing matches with point cloud
    transpose(complete_label_img,complete_label_img);
    flip(complete_label_img,complete_label_img,1);
    cv::imshow("complete_label_img", complete_label_img);
    std::string labelled_pc_anno_path = root_path + "/LABELLED_PC/LABELLED_PC_ASCII_" + std::to_string(counter) + view_id + ".pcd";
    pcl::PointCloud<pcl::PointXYZL>::Ptr show_cloud (new pcl::PointCloud<pcl::PointXYZL> ());
    changeColor(complete_label_img, cloud_xyzrgba_ptr, show_cloud);
    pcl::io::savePCDFileASCII(labelled_pc_anno_path, *show_cloud);
    if (!viewer->updatePointCloud (show_cloud, "cloud"))
    {
      viewer->addPointCloud(show_cloud);
    }

    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud seg");

    viewer->spinOnce (1);
    //boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    counter++;
    // check if final cloud has been reached
    if(counter == 151) // 500
    {
      // exit if final cloud was reached
      break;
      //reset counter to run through all PC again
      //counter = 0; //9
    }
    //cv::waitKey(0);
    int k = cv::waitKey(50);
    if(k == 'q')
    {
      break;
    }
  }
  return 0;
}
