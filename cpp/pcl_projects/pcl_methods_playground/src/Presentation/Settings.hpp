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
  bool setVariables(int mode);
  void resetWindow();

  //PassThrough
  float zmin = 0.0;
  float zmax = 0.37; //capture 2
  float xmin = -0.1;
  float xmax = 0.1;
  float ymin = -0.1;
  float ymax = 0.1;
  int zmax_slider = zmax*100;
  int ymax_slider = ymax*100;
  
  //smoothing
  enum filterModes {NOFILTERING, MLS, median, NUMBER_OF_FILTERS};
  int smoothingFilterSwitch = MLS;
  int smoothingFilterSwitch_slider = smoothingFilterSwitch;
  //Median filter z
  int median_windowSize = 0;
  int median_windowSize_slider = 0;
  //MLS smoothing
  float MLS_searchRadius = 0.005; //sphere radius for the k-nearest neighbors used for fitting
  int MLS_searchRadius_slider = MLS_searchRadius * 1000;

  //Normals
  float normalSearchRadius = 0.005;
  int normalSearchRadius_slider = normalSearchRadius*1000;
  int levelth = 5; //every levelth point's normal is shown
  double arrowScale = 0.005; //length of normal arrow
  int arrowScale_slider = arrowScale*1000;
  int levelth_slider = levelth;

  //smoothed normals
  enum normalPreprocessingModes {NONSMOOTHED, SMOOTHED, NUMBER_OF_NORMAL_MODES};
  int normalPreprocessingSwitch = NONSMOOTHED;
  int normalPreprocessingSwitch_slider = normalPreprocessingSwitch;
  enum normalColorMapModes {NOMAPPING, STANDARD, ABSOLUTE, NUMBER_OF_NORMAL_MAP_MODES};
  int normalColorMapModeSwitch = STANDARD;
  int normalColorMapModeSwitch_slider = normalColorMapModeSwitch;

  //RGS
  int numberOfNeighbours = 220;
  int numberOfNeighbours_slider = numberOfNeighbours/10;
  float smoothnessThreshold = 2.8;
  int smoothnessThreshold_slider = smoothnessThreshold*10;
  float curvatureThreshold = 3.5;
  int curvatureThreshold_slider = curvatureThreshold*10;

  //SVC
  enum SVCModes {VC, LV, VCLV, CV, CVREFINED, NUMBER_OF_SVCMODES};
  int SVCModeSwitch = CV;
  int SVCModeSwitch_slider = SVCModeSwitch;

  float voxel_resolution = 0.001f; //Set the resolution of the octree voxels.
  int voxel_resolution_slider = voxel_resolution*1000;
  float seed_resolution = 0.008f; //Set the resolution of the octree seed voxels.
  int seed_resolution_slider = seed_resolution*1000;
  float color_importance = 0.2f; //Set the importance of color for supervoxels.
  int color_importance_slider = color_importance*10;
  float spatial_importance = 0.4f; //Set the importance of spatial distance for supervoxels.
  int spatial_importance_slider = spatial_importance*10;
  float normal_importance = 1.0f; //Set the importance of scalar normal product for supervoxels.
  int normal_importance_slider = normal_importance*10;
  int refinement_itr = 3;
  int refinement_itr_slider = refinement_itr;

  //LCCP
  float concavity_tolerance_threshold = 10; //Angle threshold for concave edges to be treated as convex.
  int concavity_tolerance_threshold_slider = concavity_tolerance_threshold;
  float smoothness_threshold = 0.2; //Invalidate steps. Value from the interval [0,1], where 0 is the strictest and 1 equals 'no smoothness check'
  int smoothness_threshold_slider = smoothness_threshold*10;
  uint32_t min_segment_size = 4; //Merge small segments which have fewer points than minimal segment size
  int min_segment_size_slider = min_segment_size;
  int use_extended_convexity = 0;
  int use_extended_convexity_slider = use_extended_convexity;
  int use_sanity_criterion = 1;
  int use_sanity_criterion_slider = use_sanity_criterion;

  //CPC
  //Plane cutting parameters for splitting of segments
  float min_cut_score = 0.16; //Minumum score a proposed cut needs to have for being cut (default 0.16)
  int min_cut_score_slider = min_cut_score*100;
  unsigned int max_cuts = 25; //Perform cuts up to this recursion level. Cuts are performed in each segment separately (default 25)
  int max_cuts_slider = max_cuts;
  unsigned int cutting_min_segments = 100; //Minumum number of supervoxels in the segment to perform cutting (default 400)
  int cutting_min_segments_slider = cutting_min_segments/10;
  int use_local_constrain = 1;
  int use_local_constrain_slider = use_local_constrain;
  int use_directed_cutting = 1; //Use directed weigths (recommended flag)
  int use_directed_cutting_slider = use_directed_cutting;
  int use_clean_cutting = 1; //Flag set: Only split edges with supervoxels on opposite sites of the cutting-plane
  int use_clean_cutting_slider = use_clean_cutting; //Flag not set: Split all edges whose centroid is within the seed resolution distance to the cutting-plane
  unsigned int ransac_iterations = 10000; //Sets the maximum number of iterations for the RANSAC algorithm (default 10000)
  int ransac_iterations_slider = ransac_iterations/1000;

  bool update = false;
  bool presentationUpdate = false;

private:
  //bool on_trackbar(int, void*);

  void setAxes();
	int axis_slider_max = 100; // is / with 100

  void setSmoothingFilterType();
  int smoothingFilterSwitch_max = 2;

  void setMLSSettings();
  int MLS_searchRadius_slider_max = 10;

  void setMedianFilterSettings();
  int median_windowSize_slider_max = 25;

  void setNormalSettings();
  int normalSearchRadius_slider_max = 10;
  int normalPreprocessingSwitch_slider_max = 1;
  int normalColorMapModeSwitch_slider_max = 2;

  void setNormalPressentationSettings();
  int levelth_slider_max = 50;
  int arrowScale_slider_max = 10; //is / with 100

  //RGS
  void setRGSSettings();
  int numberOfNeighbours_slider_max = 50; // is * with 10
  int curvatureThreshold_slider_max = 50; // is / with 100
  int smoothnessThreshold_slider_max = 50; // is / with 100
  //SVC
  void setSVCSettings();
  int SVCModeSwitch_max = NUMBER_OF_SVCMODES-1;
  int voxel_resolution_max = 15;
  int seed_resolution_max = 50;
  int color_importance_max = 20;
  int spatial_importance_max = 20;
  int normal_importance_max = 20;
  int refinement_itr_max = 5;
  // LCCP
  void setLCCPSettings();
  int concavity_tolerance_threshold_max = 20;
  int smoothness_threshold_max = 10;
  int min_segment_size_max = 10;
  int use_extended_convexity_max = 1;
  int use_sanity_criterion_max = 1;
  //CPC
  void setCPCSettings();
  int min_cut_score_max = 30;
  int max_cuts_max = 50;
  int cutting_min_segments_max = 20;
  int use_local_constrain_max = 1;
  int use_directed_cutting_max = 1;
  int use_clean_cutting_max = 1;
  int ransac_iterations_max = 20;

  char TrackbarName[50];

};
