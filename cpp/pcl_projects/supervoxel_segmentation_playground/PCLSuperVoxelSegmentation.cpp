#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "defines.hpp"
#include "src/SVC.hpp"
#include "src/Settings.hpp"
#include "src/filters.hpp"
#include "src/Graphics.hpp"
#include "src/FeatureExtractor.hpp"
#include "src/Normals.hpp"

// Toggle whether to automatically proceed with processing the next point cloud
bool auto_continue = false;

int main (int argc, char ** argv)
{
  // Settings object stores configuration for the various algorithms and the visualization
  Settings Settings;
  // Supervoxel object for segmenting point cloud into supervoxels
  SVC SVC;
  // Filter object for preprocessing point cloud
  Filters filters;
  // Presenter object for drawing and visualization
  Graphics Graphics;
  // Feature extractor object for extrating features from supervoxels
  FeatureExtractor FeatureExtractor;
  // Normals object for computing point cloud normals
  Normals Normals;
  // Setup and initialize viewer object
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = Graphics.initViewer();
  // Declear point cloud variables
  pcl::PointCloud<PointT>::Ptr cloud_xyzrgb_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_xyzrgb_filtered_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointTL>::Ptr cloud_anno_ptr (new pcl::PointCloud<PointTL>);
  // PC id for iterating through input point cloud and annotation files
  unsigned int pointcloud_number = 0;
  // Loop for interactive viewing
  while (!viewer->wasStopped ())
  {
    // Check if a new point cloud has been loaded and new computations are necessary
    if(Settings.newPointCloud)
    {
      // Open an output stream for the current point cloud
      FeatureExtractor.openOutputStream(pointcloud_number);
      // Create path strings for input files
      std::string path_pointcloud = "cloud_0001.pcd";

      // Load input files
      pcl::console::print_highlight ("Loading point cloud...\n");
      if (pcl::io::loadPCDFile<PointT> (path_pointcloud, *cloud_xyzrgb_ptr))
      {
        pcl::console::print_error ("Error loading cloud file!\n");
        return (1);
      }
      if(Settings.feature_extract)
      {
        std::string path_annotations = "../input/labelled/PC_ASCII_LABELLED_" + std::to_string(pointcloud_number) + "_A.pcd";

        pcl::console::print_highlight ("Loading labelled point cloud...\n");
        if (pcl::io::loadPCDFile<PointTL> (path_annotations, *cloud_anno_ptr))
        {
          pcl::console::print_error ("Error loading cloud file!\n");
          return (1);
        }
      }

      // Remove extreme points from point cloud in preparation for segmentation
      filters.pass_through(cloud_xyzrgb_ptr, cloud_xyzrgb_filtered_ptr, 0.5, 1.5, 0.0, 0.4, -0.5, 1.5);
    }
    // Check if settings have been changed and new computations are necessary
    if(Settings.settingsUpdated || Settings.newPointCloud)
    {
      Settings.settingsUpdated = false;

      SVC.clearOldPolygons(viewer);
      SVC.segmentPC(cloud_xyzrgb_filtered_ptr, Settings.voxel_resolution, Settings.seed_resolution, Settings.color_importance, Settings.spatial_importance, Settings.normal_importance, Settings.refinement_itr);

      // Adding elements to the viewer based on the settings selection
      if(Settings.AdjacencyGraphSwitch == Settings::ADJACENCYGRAPH)
      {
        SVC.drawAdjacencyGraph(viewer);
      }
      if(Settings.presentationUpdated)
      {
        viewer->removePointCloud("supervoxel_normals");
        Settings.presentationUpdated = false;
      }
      if(!viewer->contains("supervoxel_normals"))
      {
        std::cout << "adding supervoxel_normals" << std::endl;
        viewer->addPointCloudNormals<pcl::PointNormal> (SVC.sv_normal_cloud, Settings.levelth, Settings.arrowScale, "supervoxel_normals");
      }
      if(Settings.SVCModeSwitch == Settings::CV)
      {
        if (!viewer->updatePointCloud(SVC.voxel_colored_cloud, "cloud"))
        {
          viewer->addPointCloud(SVC.voxel_colored_cloud, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "cloud");
          std::cout << "added org labeled points" << std::endl;
        }
      }
      if(Settings.SVCModeSwitch == Settings::VC)
      {
        if (!viewer->updatePointCloud(cloud_xyzrgb_filtered_ptr, "cloud"))
        {
          viewer->addPointCloud(cloud_xyzrgb_filtered_ptr, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "cloud");
          std::cout << "added org RGB cloud" << std::endl;
        }
      }
      if(Settings.SVCModeSwitch == Settings::LV)
      {
        if (!viewer->updatePointCloud(SVC.labeled_voxel_cloud, "cloud"))
        {
          viewer->addPointCloud(SVC.labeled_voxel_cloud, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "cloud");
          std::cout << "added labeled grid points" << std::endl;
        }
      }
      if(Settings.SVCModeSwitch == Settings::VCLV)
      {
        if (!viewer->updatePointCloud(SVC.voxel_centroid_cloud, "cloud"))
        {
          viewer->addPointCloud(SVC.voxel_centroid_cloud, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "cloud");
          std::cout << "added RGB grid points" << std::endl;
        }
      }
      if(Settings.SVCModeSwitch == Settings::CVREFINED)
      {
        if (!viewer->updatePointCloud(SVC.voxel_colored_cloud_refined, "cloud"))
        {
          viewer->addPointCloud(SVC.voxel_colored_cloud_refined, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "cloud");
          std::cout << "added refined labeled points" << std::endl;
        }
      }
      if(Settings.SVCModeSwitch == Settings::SUPERVOXEL)
      {
        if(viewer->contains("cloud"))
        {
          viewer->removePointCloud("cloud");
        }
        Settings.ShowUptoVoxels_max = SVC.supervoxel_clusters.size();
        SVC.clearOldSupervoxels(viewer);
        SVC.showSuperVoxels(viewer, Settings.ShowUptoVoxels);
      }
      // Extract features when a new point cloud has been processed
      if(Settings.newPointCloud && Settings.feature_extract)
      {
        FeatureExtractor.extractSuperVoxelFeatures(cloud_anno_ptr, SVC.supervoxel_clusters, SVC.supervoxel_adjacency, pointcloud_number);
      }
    }
    Settings.setVariables(); // Show the settings sliders
    viewer->spinOnce(10); // Give the viewer time to draw
    //cv::waitKey();
    if(auto_continue)
    {
      // Close the output stream for the processed point cloud
      FeatureExtractor.endOutputStream();
      pointcloud_number++;
      // 150 Break at the final ID
      if(pointcloud_number > 150)
      {
        break;
      }
      // Prepared to process and view a new point cloud
      Settings.settingsUpdated = true;
      Settings.presentationUpdated = true;
      Settings.newPointCloud = true;
    }
    else
    {
      Settings.newPointCloud = false;
    }
    // Wait for keypresses for 50ms
    int k = cv::waitKey(50);
    // Quit application
    if (k=='q')
    {
      break;
    }
    // Continue to next point cloud
    if (k=='c')
    {
      auto_continue = !auto_continue;
    }
  }
  return (0);
}
