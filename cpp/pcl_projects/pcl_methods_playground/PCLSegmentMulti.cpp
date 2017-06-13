#include <boost/thread/thread.hpp>
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

#import "defines.hpp"

#include "src/Filters/PassThrough.hpp"
#include "src/Filters/Smoothing.hpp"

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

//#include "../src/Registration/NDT.hpp"
//#include "../src/Registration/ICP.hpp"

#include "src/conversion/NormalMap.hpp"
#include "src/conversion/Conversion.hpp"

#include <string>
#include <iostream>
#include <vector>

bool update = true;
bool quitBool = false;
int frameCounter = 0;

std::vector<std::string> modeNames = {"PassThroughMode", "smoothingMode", "NormalsMode", "CRGSMode", "RGSMode", "colorDepthMode", "SVCMode", "LCCPMode", "CPCMode"};
int modeCode = PassThroughMode;

int main ()
{
  PassThrough PassThrough;
  Smoothing Smoothing;
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
  Graphics Graphics;
  //NDT NDT;
  //ICP ICP;
  Conversion Conversion;

  // Data containers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_r (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_filtered_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_filtered_r_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_f (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_r (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
  //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = Graphics.initViewer();
  std::string path, path2, imgpath;
  cv::Mat img;
  while (!viewer->wasStopped ())
  {
    //Users/markpp/Desktop/code/data/DanpoData/CENTER_B/COLOR_PC/COLOR_PC_BIN_0_C_B.pcd 
    path = "cloud_0001.pcd";
    //path2 = "/Users/markpp/Desktop/code/data/SecondDanpoCapture/right/pointcloud/PC_BIN_" + std::to_string(frameCounter) + "_A.pcd";
    imgpath = "img_0001.png";
    /*
    if(frameCounter%2==0)
    {
      path = "/Volumes/USB\ DISK/setup\ test/output\ box\ registration/pointcloud/PC_BIN_" + std::to_string(frameCounter/2) + "_A.pcd";
    }
    else
    {
      path = "/Volumes/USB\ DISK/setup\ test/output\ box\ registration/pointcloud/PC_BIN_" + std::to_string(frameCounter/2) + "_B.pcd";
    }
    */
    img = cv::imread(imgpath);
    cv::UMat umat = img.getUMat(cv::ACCESS_READ);
    std::string setnumber = "set number: " + std::to_string(frameCounter);
    cv::putText(img, setnumber, cv::Point(40,40), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3,3);
    cv::imshow("mapped", umat);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (path, *cloud_xyzrgba_ptr) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
    /*
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path2, *cloud_xyzrgb_r) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
    */
    //Convert from PointXYZRGBA to PointXYZRGB
    pcl::copyPointCloud(*cloud_xyzrgba_ptr, *cloud_xyzrgb);
    //Convert from PointXYZRGB to PointXYZ
    //pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);

    std::cout << "PointCloud " + std::to_string(frameCounter) + " has: " << cloud_xyzrgb->points.size() << " data points. Height: " << cloud_xyzrgb->height << " width: " << cloud_xyzrgb->width << std::endl;
    PassThrough.filter(cloud_xyzrgb, cloud_xyzrgb_filtered_ptr, 0.5, 1.5, 0.0, 0.4, -0.5, 1.5);
    std::cout << "PointCloud " + std::to_string(frameCounter) + " After filtering, Height: " << cloud_xyzrgb_filtered_ptr->height << " width: " << cloud_xyzrgb_filtered_ptr->width << std::endl;
    //PassThrough.filter(cloud_xyzrgb_r, cloud_xyzrgb_filtered_r_ptr, Settings.zmin, Settings.zmax, Settings.xmin, Settings.xmax, Settings.ymin, Settings.ymax);

    while (!viewer->wasStopped ())
    {
      if(update)
      {
        if(modeCode%numModes == smoothingMode || modeCode%numModes == NormalsMode) // 1 = median smoothing
        {
          if((modeCode%numModes == smoothingMode && Settings.smoothingFilterSwitch == Settings::MLS) || (modeCode%numModes == NormalsMode && Settings.normalPreprocessingSwitch == Settings::SMOOTHED))
          {
            //std::cout << Settings.MLS_searchRadius << std::endl;
            Smoothing.MLSSmoothing(cloud_xyzrgb_filtered_ptr, Settings.MLS_searchRadius);
            std::cout << "MLS" << std::endl;
          }
          else if(Settings.smoothingFilterSwitch == Settings::median)
          {
            Smoothing.medianFilter(cloud_xyzrgba_ptr, Settings.median_windowSize);
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
            Normals.generateNormals(Smoothing.smoothed_MLS_cloud_ptr, Settings.normalSearchRadius);
            Conversion.appendNormals(Smoothing.smoothed_MLS_cloud_ptr, Normals.normals_ptr);
            NormalCoding.colorNormals(Conversion.cloud_normals_ptr, Settings.normalColorMapModeSwitch);
            //NormalCoding.colorNormals(Smoothing.smoothed_MLS_normal_cloud_ptr, Settings.normalColorMapModeSwitch);
          }

        }
        if(modeCode%numModes == CRGSMode) // 1 = colored segmentation
        {
          CRGS.segmentPC(cloud_xyzrgb_filtered_ptr);
        }
        /*
        if(modeCode%numModes == PrincipalCurveMode) //
        {
          pcl::copyPointCloud(*cloud_xyzrgb_filtered_ptr, *cloud_xyz_f);
          //pcl::copyPointCloud(*cloud_xyzrgb_filtered_r_ptr, *cloud_xyz_r);
          //NDT.registerCloudPair(cloud_xyz_f, cloud_xyz_r);
          PrincipalCurves.generatePrincipalCurves(cloud_xyz_f);
        }
        */
        if(modeCode%numModes == colorDepthMode) // 3 = depth colored
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
                                Settings.ransac_iterations
                                );
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
        //
        if(Settings.normalColorMapModeSwitch == Settings::NOMAPPING)
        {
          if(Settings.normalPreprocessingSwitch == Settings::SMOOTHED)
          {
            if (!viewer->updatePointCloud (Smoothing.smoothed_MLS_cloud_ptr, "cloud"))
            {
              viewer->addPointCloud(Smoothing.smoothed_MLS_cloud_ptr, "cloud");
              //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_xyzrgb, normals_ptr, 40, 0.01, "normals");
            }
            if(!viewer->contains("normals"))
            {
              std::cout << "adding normals" << std::endl;
              viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (Smoothing.smoothed_MLS_cloud_ptr, Normals.normals_ptr, Settings.levelth, Settings.arrowScale, "normals");
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
      /*
      else if(modeCode%numModes == PrincipalCurveMode)
      {
        if (!viewer->updatePointCloud (cloud_xyz_f, "cloud"))
        {
          viewer->addPointCloud(cloud_xyz_f, "cloud");
          //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_xyzrgb, normals_ptr, 40, 0.01, "normals");
        }
        if(Settings.presentationUpdate)
        {
          viewer->removePointCloud("curves");
          Settings.presentationUpdate = false;
        }
        if(!viewer->contains("curves"))
        {
          // no curves shows up. fix?
          std::cout << "adding curves" << std::endl;
          viewer->addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal> (cloud_xyz_f, PrincipalCurves.cloudWithNormals_ptr, PrincipalCurves.principalCurvatures_ptr, 100, 1.0, "curves");
          //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "curves");

          std::cout << "added curves" << std::endl;
        }
      }
      */
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
          if (!viewer->updatePointCloud(Smoothing.smoothed_MLS_cloud_ptr, "cloud"))
          {
            viewer->addPointCloud(Smoothing.smoothed_MLS_cloud_ptr, "cloud");
          }
        }
        else if(Settings.smoothingFilterSwitch == Settings::median)
        {
          if (!viewer->updatePointCloud(Smoothing.smoothed_median_cloud_ptr, "cloud"))
          {
            viewer->addPointCloud(Smoothing.smoothed_median_cloud_ptr, "cloud");
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

      /*
      else if(modeCode%numModes == NDTMode)
      {

        if (!viewer->updatePointCloud (cloud_xyz_f, "target cloud"))
        {
          // Coloring and visualizing target cloud (red).
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          target_color (cloud_xyz_f, 255, 0, 0);
          viewer->addPointCloud<pcl::PointXYZ> (cloud_xyz_f, target_color, "target cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
        }
        if (!viewer->updatePointCloud (NDT.transformed_input_cloud_ptr, "output cloud"))
        {
          // Coloring and visualizing transformed input cloud (green).
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          output_color (NDT.transformed_input_cloud_ptr, 0, 255, 0);
          viewer->addPointCloud<pcl::PointXYZ> (NDT.transformed_input_cloud_ptr, output_color, "output cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
          std::cout << "aa" << std::endl;
        }

      }
      else if(modeCode%numModes == ICPMode)
      {
        //viewer->removePointCloud("normals");
        if (!viewer->updatePointCloud (cloud_xyzrgb_filtered_ptr, "target cloud"))
        {
          viewer->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb_filtered_ptr, "target cloud");
        }
        else
        {
          viewer->removePointCloud("target cloud");
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> target_color (cloud_xyzrgb_filtered_ptr, 0, 0, 150);
          if(Settings.ICPViewMode == 1 || Settings.ICPViewMode == 3)
          {
            viewer->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb_filtered_ptr, target_color, "target cloud");
          }
          else
          {
            viewer->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb_filtered_ptr, "target cloud");
          }
        }
        if (!viewer->updatePointCloud (cloud_xyzrgb_filtered_r_ptr, "output cloud"))
        {
          viewer->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb_filtered_r_ptr, "output cloud");
        }
        else
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_color (cloud_xyzrgb_filtered_r_ptr, 0, 150, 0);
          viewer->removePointCloud("output cloud");

          if(Settings.ICPViewMode == 2 || Settings.ICPViewMode == 3)
          {

            viewer->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb_filtered_r_ptr, output_color, "output cloud");
          }
          else
          {
            viewer->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb_filtered_r_ptr, "output cloud");
          }
        }
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "output cloud");
      }
      */


      viewer->removeShape("modeText", 0);
      viewer->addText(modeNames[modeCode%numModes], 10, 10, "modeText", 0);

      update = Settings.setVariables(modeCode%numModes);
      Settings.update = false;
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
        /*
        if(modeCode%numModes == PrincipalCurveMode) //
        {
          pcl::io::savePLYFileASCII("../output/principalCurvatures.ply", *PrincipalCurves.principalCurvatures_ptr);
          //pcl::io::savePCDFileASCII("../output/principalCurvatures.pcd", *PrincipalCurves.principalCurvatures_ptr);
        }
        */
        if(modeCode%numModes == colorDepthMode) // 3 = depth colored
        {
          pcl::io::savePLYFileASCII("../output/depthColored_cloud.ply", *DepthCoding.depthColored_cloud_ptr);
          //pcl::io::savePCDFileASCII("../output/depthColored_cloud.pcd", *DepthCoding.depthColored_cloud_ptr);
        }
        if(modeCode%numModes == smoothingMode) // 4 = median smoothing
        {
          if(Settings.smoothingFilterSwitch == Settings::MLS)
          {
            pcl::io::savePLYFileASCII("../output/smoothed_MLS_cloud.ply", *Smoothing.smoothed_MLS_normal_cloud_ptr);
            //pcl::io::savePCDFileASCII("../output/smoothed_median_cloud.pcd", *Smoothing.smoothed_MLS_Cloud_ptr);
          }
          else
          {
            pcl::io::savePLYFileASCII("../output/smoothed_median_cloud.ply", *Smoothing.smoothed_median_cloud_ptr);
            //pcl::io::savePCDFileASCII("../output/smoothed_median_cloud.pcd", *Smoothing.smoothed_median_cloud_ptr);
          }
          pcl::io::savePLYFileASCII("../output/smoothed_median_cloud.ply", *Smoothing.smoothed_median_cloud_ptr);
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
