#include <boost/thread/thread.hpp>
#include <string>
#include <iostream>
#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/io.h>
#include <pcl/console/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "defines.hpp"

#include "src/filters/filters.hpp"
#include "src/Segmentation/RGS.hpp"
#include "src/Segmentation/CRGS.hpp"
#include "src/Segmentation/SVC.hpp"
#include "src/Segmentation/LCCP.hpp"
#include "src/Segmentation/CPC.hpp"
#include "src/Presentation/Normals.hpp"
#include "src/Presentation/PrincipalCurves.hpp"
#include "src/Presentation/ColorCoding/DepthCoding.hpp"
#include "src/Presentation/ColorCoding/NormalCoding.hpp"
#include "src/Presentation/Settings.hpp"
#include "src/Presentation/Graphics.hpp"
#include "src/conversion/NormalMap.hpp"
#include "src/conversion/Conversion.hpp"

bool update = true;
bool quitBool = false;
int frameCounter = 0;

std::vector<std::string> modeNames = {"PassThroughMode", "smoothingMode", "NormalsMode", "CRGSMode", "RGSMode", "colorDepthMode", "SVCMode", "LCCPMode", "CPCMode"};
int modeCode = PassThroughMode;

int main ( int argc, char** argv )
{
  Filters filters;
  RGS RGS;
  CRGS CRGS;
  SVC SVC;
  LCCP LCCP;
  CPC CPC;
  Normals Normals;
  PrincipalCurves PrincipalCurves;
  DepthCoding DepthCoding;
  NormalCoding NormalCoding;
  Settings Settings;
  Graphics graphics;
  Conversion conversion;

  // Data containers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_filtered_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = graphics.initViewer(0.0, 0.05, -0.5, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);

  std::string path, path2, imgpath;
  cv::Mat img;
  while (!viewer->wasStopped ())
  {
    cloud_path = "cloud_0172.pcd";
    img_path = "img_0172.png";

    img = cv::imread(img_path);
    cv::UMat umat = img.getUMat(cv::ACCESS_READ);
    std::string setnumber = "set number: " + std::to_string(frameCounter);
    cv::putText(img, setnumber, cv::Point(40,40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3,3);
    cv::imshow("mapped", umat);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (cloud_path, *cloud_xyzrgba_ptr) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
    //Convert from PointXYZRGB to PointXYZ
    //pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
    filters.pass_through(cloud_xyzrgb,
                         0.0, 0.4,
                         -0.5, 1.5,
                         0.5, 1.5);
    cloud_xyzrgb_filtered_ptr = filters.filtered_cloud_ptr;

    while (!viewer->wasStopped ())
    {
      if(update)
      {
        if(modeCode%numModes == smoothingMode || modeCode%numModes == NormalsMode) // 1 = median smoothing
        {
          if((modeCode%numModes == smoothingMode && Settings.smoothingFilterSwitch == Settings::MLS) || (modeCode%numModes == NormalsMode && Settings.normalPreprocessingSwitch == Settings::SMOOTHED))
          {
            filters.MLS_smoothing(cloud_xyzrgb_filtered_ptr, Settings.MLS_searchRadius);
            std::cout << "MLS" << std::endl;
          }
          else if(Settings.smoothingFilterSwitch == Settings::median)
          {
            filters.medianFilter(cloud_xyzrgba_ptr, Settings.median_windowSize);
            std::cout << "MedianFiltered" << std::endl;
          }
        }
        if(modeCode%numModes == NormalsMode || modeCode%numModes == RGSMode) // 2 = normals
        {
          if((modeCode%numModes == NormalsMode && Settings.normalPreprocessingSwitch == Settings::NONSMOOTHED) || modeCode%numModes == RGSMode)
          {
            std::cout << "NONSMOOTHED" << std::endl;
            //With noisy normals
            Normals.generateNormals(cloud_xyzrgb_filtered_ptr, Settings.normalSearchRadius);
            Conversion.appendNormals(cloud_xyzrgb_filtered_ptr, Normals.normals_ptr);
            NormalCoding.colorNormals(Conversion.cloud_normals_ptr, Settings.normalColorMapModeSwitch);
          }
          else if((modeCode%numModes == NormalsMode && Settings.normalPreprocessingSwitch == Settings::SMOOTHED) && modeCode%numModes != RGSMode)
          {
            std::cout << "SMOOTHED" << std::endl;
            Normals.generateNormals(filters.smoothed_MLS_cloud_ptr, Settings.normalSearchRadius);
            Conversion.appendNormals(filters.smoothed_MLS_cloud_ptr, Normals.normals_ptr);
            NormalCoding.colorNormals(Conversion.cloud_normals_ptr, Settings.normalColorMapModeSwitch);
          }
        }
        if(modeCode%numModes == CRGSMode) // 1 = colored segmentation
        {
          CRGS.segmentPC(cloud_xyzrgb_filtered_ptr);
        }
        if(modeCode%numModes == colorDepthMode) // 2 = depth colored
        {
          DepthCoding.colorDepth(cloud_xyzrgb_filtered_ptr);
        }
        if(modeCode%numModes == RGSMode) // 3 = custom segmentation
        {
          RGS.segmentPC(cloud_xyzrgb_filtered_ptr, Normals.normals_ptr, Settings.numberOfNeighbours, Settings.smoothnessThreshold, Settings.curvatureThreshold);
        }
        if(modeCode%numModes == SVCMode || modeCode%numModes == LCCPMode || modeCode%numModes == CPCMode) // 3 = custom segmentation
        {
          SVC.segmentPC(cloud_xyzrgb_filtered_ptr, Settings.voxel_resolution, Settings.seed_resolution, Settings.color_importance, Settings.spatial_importance, Settings.normal_importance, Settings.refinement_itr);
        }
        if(modeCode%numModes == LCCPMode) // 3 = custom segmentation
        {
          LCCP.segmentPC(cloud_xyzrgb_filtered_ptr,
                                SVC.supervoxel_clusters,
                                SVC.supervoxel_adjacency,
                                SVC.voxel_colored_cloud,
                                Settings.voxel_resolution,
                                Settings.seed_resolution,
                                Settings.concavity_tolerance_threshold,
                                Settings.smoothness_threshold,
                                Settings.min_segment_size,
                                Settings.use_extended_convexity,
                                Settings.use_sanity_criterion);
        }
        if(modeCode%numModes == CPCMode) // 3 = custom segmentation
        {
          CPC.segmentPC(cloud_xyzrgb_filtered_ptr,
                                SVC.supervoxel_clusters,
                                SVC.supervoxel_adjacency,
                                SVC.voxel_colored_cloud,
                                Settings.voxel_resolution,
                                Settings.seed_resolution,
                                Settings.concavity_tolerance_threshold,
                                Settings.smoothness_threshold,
                                Settings.min_segment_size,
                                Settings.use_extended_convexity,
                                Settings.use_sanity_criterion,
                                Settings.min_cut_score,
                                Settings.max_cuts,
                                Settings.cutting_min_segments,
                                Settings.use_local_constrain,
                                Settings.use_directed_cutting,
                                Settings.use_clean_cutting,
                                Settings.ransac_iterations);
        }
        update = false;
      }
      if(modeCode%numModes == PassThroughMode)
      {
        if (!viewer->updatePointCloud (cloud_xyzrgb_filtered_ptr, "cloud"))
        {
          viewer->addPointCloud(cloud_xyzrgb_filtered_ptr, "cloud");
        }
        if(viewer->contains("z_plane"))
        {
          Graphics.removeBoundary_z(viewer);
        }
        if(viewer->contains("y_plane"))
        {
          Graphics.removeBoundary_y(viewer);
        }
        Graphics.drawBoundary_z(viewer, 0.05, Settings.zmax);
        Graphics.drawBoundary_y(viewer, 0.05, Settings.ymax);
      }
      else if(modeCode%numModes == CRGSMode)
      {
        viewer->removePointCloud("normals");
        if (!viewer->updatePointCloud (CRGS.crgs_segmented_cloud_ptr, "cloud"))
        {
          viewer->addPointCloud(CRGS.crgs_segmented_cloud_ptr, "cloud");
        }
      }
      else if(modeCode%numModes == NormalsMode)
      {
        if(Settings.presentationUpdate)
        {
          viewer->removePointCloud("normals");
          Settings.presentationUpdate = false;
        }
        if(Settings.normalColorMapModeSwitch == Settings::NOMAPPING)
        {
          if(Settings.normalPreprocessingSwitch == Settings::SMOOTHED)
          {
            if (!viewer->updatePointCloud (filters.smoothed_MLS_cloud_ptr, "cloud"))
            {
              viewer->addPointCloud(filters.smoothed_MLS_cloud_ptr, "cloud");
              //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_xyzrgb, normals_ptr, 40, 0.01, "normals");
            }
            if(!viewer->contains("normals"))
            {
              std::cout << "adding normals" << std::endl;
              viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (filters.smoothed_MLS_cloud_ptr, Normals.normals_ptr, Settings.levelth, Settings.arrowScale, "normals");
            }
          }
          else
          {
            if (!viewer->updatePointCloud (cloud_xyzrgb_filtered_ptr, "cloud"))
            {
              viewer->addPointCloud(cloud_xyzrgb_filtered_ptr, "cloud");
              //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_xyzrgb, normals_ptr, 40, 0.01, "normals");
            }
            if(!viewer->contains("normals"))
            {
              std::cout << "adding normals" << std::endl;
              viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_xyzrgb_filtered_ptr, Normals.normals_ptr, Settings.levelth, Settings.arrowScale, "normals");
            }
          }
        }
        else
        {
          if (!viewer->updatePointCloud (NormalCoding.normalColored_cloud_ptr, "cloud"))
          {
            viewer->addPointCloud(NormalCoding.normalColored_cloud_ptr, "cloud");
          }
          if(!viewer->contains("normals"))
          {
            std::cout << "adding normals" << std::endl;
            viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (NormalCoding.normalColored_cloud_ptr, Normals.normals_ptr, Settings.levelth, Settings.arrowScale, "normals");
          }
        }
      }
      else if(modeCode%numModes == colorDepthMode)
      {
        viewer->removePointCloud("normals");
        if (!viewer->updatePointCloud(DepthCoding.depthColored_cloud_ptr, "cloud"))
        {
          viewer->addPointCloud(DepthCoding.depthColored_cloud_ptr, "cloud");
          //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_xyzrgb, normals_ptr, 200, 0.01, "normals");
        }
      }
      else if(modeCode%numModes == smoothingMode)
      {
        if(Settings.smoothingFilterSwitch == Settings::MLS)
        {
          if (!viewer->updatePointCloud(filters.smoothed_MLS_cloud_ptr, "cloud"))
          {
            viewer->addPointCloud(filters.smoothed_MLS_cloud_ptr, "cloud");
          }
        }
        else if(Settings.smoothingFilterSwitch == Settings::median)
        {
          if (!viewer->updatePointCloud(filters.smoothed_median_cloud_ptr, "cloud"))
          {
            viewer->addPointCloud(filters.smoothed_median_cloud_ptr, "cloud");
          }
        }
        else
        {
          if (!viewer->updatePointCloud(cloud_xyzrgb_filtered_ptr, "cloud"))
          {
            viewer->addPointCloud(cloud_xyzrgb_filtered_ptr, "cloud");
          }
        }
      }
      else if(modeCode%numModes == RGSMode)
      {
        //viewer->removePointCloud("normals");
        if (!viewer->updatePointCloud (RGS.rgs_segmented_cloud_ptr, "cloud"))
        {
          viewer->addPointCloud(RGS.rgs_segmented_cloud_ptr, "cloud");
        }
        if(Settings.presentationUpdate)
        {
          viewer->removePointCloud("normals");
          Settings.presentationUpdate = false;
        }
        if(!viewer->contains("normals"))
        {
          std::cout << "adding normals" << std::endl;
          viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (RGS.rgs_segmented_cloud_ptr, Normals.normals_ptr, Settings.levelth, Settings.arrowScale, "normals");
        }
      }
      else if(modeCode%numModes == SVCMode)
      {
        if(Settings.presentationUpdate)
        {
          viewer->removePointCloud("normals");
          viewer->removePointCloud("supervoxel_normals");
          Settings.presentationUpdate = false;
        }
        if(!viewer->contains("supervoxel_normals"))
        {
          std::cout << "adding supervoxel_normals" << std::endl;
          viewer->addPointCloudNormals<pcl::PointNormal> (SVC.sv_normal_cloud, Settings.levelth, Settings.arrowScale, "supervoxel_normals");
          //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (NormalCoding.normalColored_cloud_ptr, SVC.sv_normal_cloud, Settings.levelth, Settings.arrowScale, "normals");
        }
        if(Settings.SVCModeSwitch == 0 || Settings.SVCModeSwitch == 2)
        {
          //viewer->removeAllPointClouds();
          if(viewer->contains("labeled points"))
          {
            viewer->removePointCloud("labeled points");
          }
          if(Settings.SVCModeSwitch == 0)
          {
            if(viewer->contains("labeled voxels"))
            {
              viewer->removePointCloud("labeled voxels");
            }
          }
          if (!viewer->updatePointCloud(SVC.voxel_centroid_cloud, "voxel centroids"))
          {
            viewer->addPointCloud (SVC.voxel_centroid_cloud, "voxel centroids");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "voxel centroids");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");
            std::cout << "added voxel centroids" << std::endl;
          }
        }
        if(Settings.SVCModeSwitch == 1 || Settings.SVCModeSwitch == 2)
        {
          if(viewer->contains("labeled points"))
          {
            viewer->removePointCloud("labeled points");
          }
          if(Settings.SVCModeSwitch == 1)
          {
            if(viewer->contains("voxel centroids"))
            {
              viewer->removePointCloud("voxel centroids");
            }
          }
          if (!viewer->updatePointCloud(SVC.labeled_voxel_cloud, "labeled voxels"))
          {
            viewer->addPointCloud (SVC.labeled_voxel_cloud, "labeled voxels");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "labeled voxels");
            std::cout << "added labeled voxel" << std::endl;
          }
        }
        if(Settings.SVCModeSwitch == 3)
        {
          if(viewer->contains("voxel centroids"))
          {
            viewer->removePointCloud("voxel centroids");
          }
          if(viewer->contains("labeled voxels"))
          {
            viewer->removePointCloud("labeled voxels");
          }
          if (!viewer->updatePointCloud(SVC.voxel_colored_cloud, "labeled points"))
          {
            viewer->addPointCloud (SVC.voxel_colored_cloud, "labeled points");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled points");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "labeled points");
            std::cout << "added labeled points" << std::endl;
          }
        }
        if(Settings.SVCModeSwitch == 4)
        {
          if(viewer->contains("voxel centroids"))
          {
            viewer->removePointCloud("voxel centroids");
          }
          if(viewer->contains("labeled voxels"))
          {
            viewer->removePointCloud("labeled voxels");
          }
          if (!viewer->updatePointCloud(SVC.voxel_colored_cloud_refined, "labeled points"))
          {
            viewer->addPointCloud (SVC.voxel_colored_cloud_refined, "labeled points");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled points");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1.0, "labeled points");
            std::cout << "added labeled points" << std::endl;
          }
        }
      }
      else if(modeCode%numModes == LCCPMode)
      {
        viewer->removePointCloud("normals");
        if (!viewer->updatePointCloud(LCCP.lccp_labeled_cloud_ptr, "labeled points"))
        {
          viewer->addPointCloud(LCCP.lccp_labeled_cloud_ptr, "labeled points");
          //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_xyzrgb, normals_ptr, 200, 0.01, "normals");
        }
      }
      else if(modeCode%numModes == CPCMode)
      {
        viewer->removePointCloud("normals");
        if (!viewer->updatePointCloud(CPC.cpc_labeled_cloud_ptr, "labeled points"))
        {
          viewer->addPointCloud(CPC.cpc_labeled_cloud_ptr, "labeled points");
          //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_xyzrgb, normals_ptr, 200, 0.01, "normals");
        }
      }
      update = Settings.setVariables(modeCode%numModes);
      Settings.update = false;

      viewer->removeShape("mode_text", 0);
      viewer->addText(modeNames[modeCode%numModes], 10, 10, 50, 0.0, 0.0, 1.0, "mode_text", 0);
      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
      viewer->spinOnce (1);
      boost::this_thread::sleep (boost::posix_time::microseconds (10));

      int k = cv::waitKey(50);
		  if (k=='n') {
        update = true;
			  break;
		  }
      if (k=='m') {
        //colorCoded = !colorCoded;
        Settings.resetWindow();
        if(modeCode%numModes == PassThroughMode)
        {
          Graphics.removeBoundary_z(viewer);
          Graphics.removeBoundary_y(viewer);
        }
        modeCode++;
        update = true;
        std::cout << "Mode: " << modeNames[modeCode%numModes] << std::endl;
      }
      if (k=='M') {
        Settings.resetWindow();
        if(modeCode%numModes == PassThroughMode)
        {
          Graphics.removeBoundary_z(viewer);
          Graphics.removeBoundary_y(viewer);
        }
        //colorCoded = !colorCoded;
        modeCode--;
        update = true;
        std::cout << "x: " << modeCode%numModes << endl;
      }
      if (k=='s')
      {
        std::cout << "PointCloud written to file." << std::endl;
        if(modeCode%numModes == PassThroughMode) // 0 = color
        {
          pcl::io::savePLYFileASCII("../output/cloud_xyzrgb_filtered.ply", *cloud_xyzrgb_filtered_ptr);
          //pcl::io::savePCDFileASCII("../output/cloud_xyzrgb_filtered.pcd", *cloud_xyzrgb_filtered_ptr);
        }
        if(modeCode%numModes == CRGSMode) // 1 = colored segmentation
        {
          pcl::io::savePLYFileASCII("../output/crgs_segmented_cloud.ply", *CRGS.crgs_segmented_cloud_ptr);
          //pcl::io::savePCDFileASCII("../output/crgs_segmented_cloud.pcd", *CRGS.crgs_segmented_cloud_ptr);
        }
        if(modeCode%numModes == NormalsMode || modeCode%numModes == RGSMode) // 2 = normals
        {
          pcl::io::savePLYFileASCII("../output/normals_ptr.ply", *Normals.normals_ptr);
          //pcl::io::savePCDFileASCII("../output/normals_ptr.pcd", *Normals.normals_ptr);
        }
        if(modeCode%numModes == colorDepthMode) // 3 = depth colored
        {
          pcl::io::savePLYFileASCII("../output/depthColored_cloud.ply", *DepthCoding.depthColored_cloud_ptr);
          //pcl::io::savePCDFileASCII("../output/depthColored_cloud.pcd", *DepthCoding.depthColored_cloud_ptr);
        }
        if(modeCode%numModes == smoothingMode) // 4 = median smoothing
        {
          if(Settings.smoothingFilterSwitch == Settings::MLS)
          {
            pcl::io::savePLYFileASCII("../output/smoothed_MLS_cloud.ply", *filters.smoothed_MLS_normal_cloud_ptr);
            //pcl::io::savePCDFileASCII("../output/smoothed_median_cloud.pcd", *Smoothing.smoothed_MLS_Cloud_ptr);
          }
          else
          {
            pcl::io::savePLYFileASCII("../output/smoothed_median_cloud.ply", *filters.smoothed_median_cloud_ptr);
            //pcl::io::savePCDFileASCII("../output/smoothed_median_cloud.pcd", *Smoothing.smoothed_median_cloud_ptr);
          }
          pcl::io::savePLYFileASCII("../output/smoothed_median_cloud.ply", *filters.smoothed_median_cloud_ptr);
          //pcl::io::savePCDFileASCII("../output/smoothed_median_cloud.pcd", *Smoothing.smoothed_median_cloud_ptr);
        }
        if(modeCode%numModes == RGSMode) // 3 = custom segmentation
        {
          pcl::io::savePLYFileASCII("../output/crgs_segmented_cloud.ply", *RGS.rgs_segmented_cloud_ptr);
          //pcl::io::savePCDFileASCII("../output/crgs_segmented_cloud.pcd", *RGS.rgs_segmented_cloud_ptr);
        }
        if(modeCode%numModes == SVCMode) // 3 = custom segmentation
        {
        }
        if(modeCode%numModes == LCCPMode) // 3 = custom segmentation
        {
        }
      }
      if (k=='q') {
			  quitBool = true;
		  }
    }
    frameCounter++;
    if(frameCounter == 144) // 500
    {
      frameCounter = 0; //9
    }
    if(quitBool)
    {
      break;
    }
  }
  return 0;
}
