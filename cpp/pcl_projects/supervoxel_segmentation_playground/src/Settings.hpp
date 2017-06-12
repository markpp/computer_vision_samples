#pragma once
/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../defines.hpp"

class Settings {
public:
  Settings();
  bool setVariables();
  void resetWindow();

  //PassThrough
  float zmin = 0.0;
  float zmax = 0.37; //capture 2
  float xmin = -0.1;
  float xmax = 0.1;
  float ymin = -0.1; //-0.1
  float ymax = 0.1; //0.1
  int zmax_slider = zmax*100;


  //Normals
  float normalSearchRadius = 0.005;
  int normalSearchRadius_slider = normalSearchRadius*1000;
  int levelth = 1; //every levelth point's normal is shown
  float arrowScale = 0.005; //length of normal arrow
  int arrowScale_slider = arrowScale*1000;
  int levelth_slider = levelth;

  // CV = voxel colored cloud, VC = Voxel cloud, LV = labeled voxel cloud, VCLV = voxel centroid cloud, CVREFINED = refined SV clusters, SUPERVOXEL = individual SV clusters
  enum SVCModes {CV, VC, LV, VCLV, CVREFINED, SUPERVOXEL, NUMBER_OF_SVCMODES};
  int SVCModeSwitch = CV;
  int SVCModeSwitch_slider = SVCModeSwitch;
  enum AdjacencyGraphModes {NOGRAPH, ADJACENCYGRAPH, NUMBER_OF_ADJACENCYGRAPHMODES};
  int AdjacencyGraphSwitch = NOGRAPH;
  int AdjacencyGraphSwitch_slider = AdjacencyGraphSwitch;

  int ShowUptoVoxels = 50;
  int ShowUptoVoxels_slider = ShowUptoVoxels;
  int ShowUptoVoxels_max = 200;

  float voxel_resolution = 0.001f; //Set the resolution of the octree voxels. //was 0.001f - 0.002f
  int voxel_resolution_slider = voxel_resolution*1000;
  float seed_resolution = 0.008f; //Set the resolution of the octree seed voxels. //was 0.008f - 0.012f
  int seed_resolution_slider = seed_resolution*1000;
  //float color_importance = 1.0f; //Set the importance of color for supervoxels.
  float color_importance = 0.2f; //Set the importance of color for supervoxels.
  int color_importance_slider = color_importance*10;
  //float spatial_importance = 0.2f; //Set the importance of spatial distance for supervoxels.
  float spatial_importance = 0.4f; //Set the importance of spatial distance for supervoxels.
  int spatial_importance_slider = spatial_importance*10;
  //float normal_importance = 0.0f; //Set the importance of scalar normal product for supervoxels.
  float normal_importance = 1.0f; //Set the importance of scalar normal product for supervoxels.
  int normal_importance_slider = normal_importance*10;
  int refinement_itr = 3;
  int refinement_itr_slider = refinement_itr;

  bool settingsUpdated = true;
  bool presentationUpdated = false;
  bool newPointCloud = true;
  bool feature_extract = false;

private:
  //bool on_trackbar(int, void*);

  void setAxes();
	int axis_slider_max = 100; // is / with 100

  void setNormalSettings();
  int normalSearchRadius_slider_max = 10;
  int normalPreprocessingSwitch_slider_max = 1;
  int normalColorMapModeSwitch_slider_max = 2;

  void setNormalPressentationSettings();
  int levelth_slider_max = 50;
  int arrowScale_slider_max = 10; //is / with 100

  void setSVCSettings();
  int SVCModeSwitch_max = NUMBER_OF_SVCMODES-1;
  int AdjacencyGraphSwitch_slider_max = NUMBER_OF_ADJACENCYGRAPHMODES-1;
  void setSVCNumberOfSupervoxels();
  int voxel_resolution_max = 15;
  int seed_resolution_max = 50;
  int color_importance_max = 20;
  int spatial_importance_max = 20;
  int normal_importance_max = 20;
  int refinement_itr_max = 5;

  char TrackbarName[50];
};
