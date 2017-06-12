#include "Settings.hpp"

Settings::Settings()
{
  cv::namedWindow("Settings window", CV_WINDOW_NORMAL);
  cv::resizeWindow("Settings window", 400, 80);
  cv::moveWindow("Settings window", 20, 20);
}

/**
 * @function on_trackbar
 * @brief Callback for trackbar
 */
void on_trackbar( int, void* )
{
}

void Settings::resetWindow()
{
  cv::destroyWindow("Settings window");
  cv::namedWindow("Settings window", CV_WINDOW_NORMAL);
  cv::resizeWindow("Settings window", 400, 80);
  cv::moveWindow("Settings window", 20, 20);
}

bool Settings::setVariables(int mode)
{
  /// Create Sliders
  if(mode == PassThroughMode)
  {
    setAxes();
  }
  if(mode == smoothingMode)
  {
    setSmoothingFilterType();
    if(smoothingFilterSwitch == MLS)
    {
      setMLSSettings();
    }
    else if(smoothingFilterSwitch == median)
    {
      setMedianFilterSettings();
    }
  }
  if(mode == NormalsMode)
  {
    setNormalSettings();
    if(smoothingFilterSwitch == MLS)
    {
      setMLSSettings();
    }
    setNormalPressentationSettings();
  }
  if(mode == RGSMode)
  {
    setRGSSettings();
  }
  if(mode == SVCMode)
  {
    setSVCSettings();
    setNormalPressentationSettings();
  }
  if(mode == LCCPMode)
  {
    setLCCPSettings();
  }
  if(mode == CPCMode)
  {
    setLCCPSettings();
    setCPCSettings();
  }
  return update;
}

void Settings::setAxes()
{
  /// Create Trackbar
  sprintf( TrackbarName, "Z-axis max");
  cv::createTrackbar( TrackbarName, "Settings window", &zmax_slider, axis_slider_max, on_trackbar );
  /// Show some stuff
  if(((float)zmax_slider/100) != zmax)
  {
    zmax = ((float)zmax_slider/100);
    std::cout << "zmax: " << zmax << std::endl;
    update = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "Y-axis max");
  cv::createTrackbar( TrackbarName, "Settings window", &ymax_slider, axis_slider_max, on_trackbar );
  /// Show some stuff
  if(((float)ymax_slider/100) != ymax)
  {
    ymax = ((float)ymax_slider/100);
    std::cout << "ymax: " << ymax << std::endl;
    update = true;
  }
}

void Settings::setSmoothingFilterType()
{
  /// Create Trackbar
  sprintf( TrackbarName, "Smoothing filter type");
  cv::createTrackbar( TrackbarName, "Settings window", &smoothingFilterSwitch_slider, smoothingFilterSwitch_max, on_trackbar );
  /// Show some stuff
  if(smoothingFilterSwitch_slider != smoothingFilterSwitch)
  {
    resetWindow();
    smoothingFilterSwitch = smoothingFilterSwitch_slider;
    std::cout << "smoothingFilterSwitch: " << smoothingFilterSwitch << std::endl;

    update = true;
  }
}

void Settings::setMLSSettings()
{
  /// Create Trackbar
  sprintf( TrackbarName, "MLS filter search radius");
  cv::createTrackbar( TrackbarName, "Settings window", &MLS_searchRadius_slider, MLS_searchRadius_slider_max, on_trackbar );
  /// Show some stuff
  if(MLS_searchRadius_slider < 1)
  {
    MLS_searchRadius_slider = 1;
  }
  if(((float)MLS_searchRadius_slider/1000) != MLS_searchRadius)
  {
    MLS_searchRadius = (float)MLS_searchRadius_slider/1000;
    std::cout << "MLS_searchRadius: " << MLS_searchRadius << std::endl;
    update = true;
  }
}

void Settings::setMedianFilterSettings()
{
  /// Create Trackbar
  sprintf( TrackbarName, "Median filter window size");
  cv::createTrackbar( TrackbarName, "Settings window", &median_windowSize_slider, median_windowSize_slider_max, on_trackbar );
  /// Show some stuff
  if(median_windowSize_slider != median_windowSize)
  {
    median_windowSize = median_windowSize_slider;
    std::cout << "median_windowSize: " << median_windowSize << std::endl;
    update = true;
  }
}

void Settings::setNormalSettings()
{
  /// Create Trackbar
  sprintf( TrackbarName, "Normal search radius");
  cv::createTrackbar( TrackbarName, "Settings window", &normalSearchRadius_slider, normalSearchRadius_slider_max, on_trackbar );
  /// Show some stuff
  if(((float)normalSearchRadius_slider/1000) != normalSearchRadius)
  {
    normalSearchRadius = ((float)normalSearchRadius_slider/1000);
    std::cout << "normalSearchRadius: " << normalSearchRadius << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "Normal preprocessing switch");
  cv::createTrackbar( TrackbarName, "Settings window", &normalPreprocessingSwitch_slider, normalPreprocessingSwitch_slider_max, on_trackbar );
  /// Show some stuff
  if(normalPreprocessingSwitch_slider != normalPreprocessingSwitch)
  {
    resetWindow();
    normalPreprocessingSwitch = normalPreprocessingSwitch_slider;
    std::cout << "normalPreprocessingSwitch: " << normalPreprocessingSwitch << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "Normal color coding mode");
  cv::createTrackbar( TrackbarName, "Settings window", &normalColorMapModeSwitch_slider, normalColorMapModeSwitch_slider_max, on_trackbar );
  /// Show some stuff
  if(normalColorMapModeSwitch_slider != normalColorMapModeSwitch)
  {
    normalColorMapModeSwitch = normalColorMapModeSwitch_slider;
    std::cout << "normalColorMapModeSwitch: " << normalColorMapModeSwitch << std::endl;
    update = true;
  }
}

void Settings::setNormalPressentationSettings()
{
  /// Create Trackbar
  sprintf( TrackbarName, "Arrow length");
  cv::createTrackbar( TrackbarName, "Settings window", &arrowScale_slider, arrowScale_slider_max, on_trackbar );
  /// Show some stuff
  if(((float)arrowScale_slider/1000) != arrowScale)
  {
    arrowScale = ((float)arrowScale_slider/1000);
    std::cout << "arrowScale: " << arrowScale << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "Levelth point");
  cv::createTrackbar( TrackbarName, "Settings window", &levelth_slider, levelth_slider_max, on_trackbar );
  /// Show some stuff
  if(levelth_slider < 1)
  {
    levelth_slider = 1;
  }
  if(levelth_slider != levelth)
  {
    levelth = levelth_slider;
    std::cout << "levelth: " << levelth << std::endl;
    update = true;
    presentationUpdate = true;
  }
}

void Settings::setRGSSettings()
{
  /// Create Trackbar
  sprintf( TrackbarName, "Curvature Threshold");
  cv::createTrackbar( TrackbarName, "Settings window", &curvatureThreshold_slider, curvatureThreshold_slider_max, on_trackbar );
  /// Show some stuff
  if(((float)curvatureThreshold_slider/10) != curvatureThreshold)
  {
    curvatureThreshold = ((float)curvatureThreshold_slider/10);
    std::cout << "curvatureThreshold: " << curvatureThreshold << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "Smoothness Threshold");
  cv::createTrackbar( TrackbarName, "Settings window", &smoothnessThreshold_slider, smoothnessThreshold_slider_max, on_trackbar );
  /// Show some stuff
  if(((float)smoothnessThreshold_slider/10) != smoothnessThreshold)
  {
    smoothnessThreshold = ((float)smoothnessThreshold_slider/10);
    std::cout << "smoothnessThreshold: " << smoothnessThreshold << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "Number Of Neighbours");
  cv::createTrackbar( TrackbarName, "Settings window", &numberOfNeighbours_slider, numberOfNeighbours_slider_max, on_trackbar );
  /// Show some stuff
  if((numberOfNeighbours_slider*10) != numberOfNeighbours)
  {
    numberOfNeighbours = (numberOfNeighbours_slider*10);
    std::cout << "numberOfNeighbours: " << numberOfNeighbours << std::endl;
    update = true;
    presentationUpdate = true;
  }
}
void Settings::setSVCSettings()
{
  /// Create Trackbar
  sprintf( TrackbarName, "SVC SVCModeSwitch");
  cv::createTrackbar( TrackbarName, "Settings window", &SVCModeSwitch_slider, SVCModeSwitch_max, on_trackbar );
  /// Show some stuff
  if(SVCModeSwitch_slider != SVCModeSwitch)
  {
    SVCModeSwitch = SVCModeSwitch_slider;
    std::cout << "SVCModeSwitch: " << SVCModeSwitch << std::endl;
    //update = true;
    presentationUpdate = true;
  }

  /// Create Trackbar
  sprintf( TrackbarName, "SVC voxel_resolution");
  cv::createTrackbar( TrackbarName, "Settings window", &voxel_resolution_slider, voxel_resolution_max, on_trackbar );
  /// Show some stuff
  if(((float)voxel_resolution_slider/1000) != voxel_resolution)
  {
    voxel_resolution = ((float)voxel_resolution_slider/1000);
    std::cout << "voxel_resolution: " << voxel_resolution << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "SVC seed_resolution");
  cv::createTrackbar( TrackbarName, "Settings window", &seed_resolution_slider, seed_resolution_max, on_trackbar );
  /// Show some stuff
  if(((float)seed_resolution_slider/1000) != seed_resolution)
  {
    seed_resolution = ((float)seed_resolution_slider/1000);
    std::cout << "seed_resolution: " << seed_resolution << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "SVC color_importance");
  cv::createTrackbar( TrackbarName, "Settings window", &color_importance_slider, color_importance_max, on_trackbar );
  /// Show some stuff
  if(((float)color_importance_slider/10) != color_importance)
  {
    color_importance = ((float)color_importance_slider/10);
    std::cout << "color_importance: " << color_importance << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "SVC spatial_importance");
  cv::createTrackbar( TrackbarName, "Settings window", &spatial_importance_slider, spatial_importance_max, on_trackbar );
  /// Show some stuff
  if(((float)spatial_importance_slider/10) != spatial_importance)
  {
    spatial_importance = ((float)spatial_importance_slider/10);
    std::cout << "spatial_importance: " << spatial_importance << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "SVC normal_importance");
  cv::createTrackbar( TrackbarName, "Settings window", &normal_importance_slider, normal_importance_max, on_trackbar );
  /// Show some stuff
  if(((float)normal_importance_slider/10) != normal_importance)
  {
    normal_importance = ((float)normal_importance_slider/10);
    std::cout << "normal_importance: " << normal_importance << std::endl;
    update = true;
    presentationUpdate = true;
  }
  if(SVCModeSwitch == CVREFINED)
  {
    /// Create Trackbar
    sprintf( TrackbarName, "SVC refinement_itr");
    cv::createTrackbar( TrackbarName, "Settings window", &refinement_itr_slider, refinement_itr_max, on_trackbar );
    /// Show some stuff
    if(refinement_itr_slider != refinement_itr)
    {
      refinement_itr = refinement_itr_slider;
      std::cout << "refinement_itr: " << refinement_itr << std::endl;
      update = true;
      presentationUpdate = true;
    }
  }
}
void Settings::setLCCPSettings()
{
  /// Create Trackbar
  sprintf( TrackbarName, "concavity_tolerance_threshold");
  cv::createTrackbar( TrackbarName, "Settings window", &concavity_tolerance_threshold_slider, concavity_tolerance_threshold_max, on_trackbar );
  /// Show some stuff
  if(concavity_tolerance_threshold_slider != concavity_tolerance_threshold)
  {
    concavity_tolerance_threshold = concavity_tolerance_threshold_slider;
    std::cout << "concavity_tolerance_threshold: " << concavity_tolerance_threshold << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "smoothness_threshold");
  cv::createTrackbar( TrackbarName, "Settings window", &smoothness_threshold_slider, smoothness_threshold_max, on_trackbar );
  /// Show some stuff
  if(((float)smoothness_threshold_slider/10) != smoothness_threshold)
  {
    smoothness_threshold = ((float)smoothness_threshold_slider/10);
    std::cout << "smoothness_threshold: " << smoothness_threshold << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "min_segment_size");
  cv::createTrackbar( TrackbarName, "Settings window", &min_segment_size_slider, min_segment_size_max, on_trackbar );
  /// Show some stuff
  if(min_segment_size_slider != min_segment_size)
  {
    min_segment_size = min_segment_size_slider;
    std::cout << "min_segment_size: " << min_segment_size << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "use_extended_convexity");
  cv::createTrackbar( TrackbarName, "Settings window", &use_extended_convexity_slider, use_extended_convexity_max, on_trackbar );
  /// Show some stuff
  if(use_extended_convexity_slider != use_extended_convexity)
  {
    use_extended_convexity = use_extended_convexity_slider;
    std::cout << "use_extended_convexity: " << use_extended_convexity << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "use_sanity_criterion");
  cv::createTrackbar( TrackbarName, "Settings window", &use_sanity_criterion_slider, use_sanity_criterion_max, on_trackbar );
  /// Show some stuff
  if(use_sanity_criterion_slider != use_sanity_criterion)
  {
    use_sanity_criterion = use_sanity_criterion_slider;
    std::cout << "use_sanity_criterion: " << use_sanity_criterion << std::endl;
    update = true;
    presentationUpdate = true;
  }
}
void Settings::setCPCSettings()
{
  /// Create Trackbar
  sprintf( TrackbarName, "min_cut_score");
  cv::createTrackbar( TrackbarName, "Settings window", &min_cut_score_slider, min_cut_score_max, on_trackbar );
  /// Show some stuff
  if(((float)min_cut_score_slider/100) != min_cut_score)
  {
    min_cut_score = ((float)min_cut_score_slider/100);
    std::cout << "min_cut_score: " << min_cut_score << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "max_cuts");
  cv::createTrackbar( TrackbarName, "Settings window", &min_cut_score_slider, min_cut_score_max, on_trackbar );
  /// Show some stuff
  if(max_cuts_slider != max_cuts)
  {
    max_cuts = max_cuts_slider;
    std::cout << "max_cuts: " << max_cuts << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "cutting_min_segments");
  cv::createTrackbar( TrackbarName, "Settings window", &cutting_min_segments_slider, cutting_min_segments_max, on_trackbar );
  /// Show some stuff
  if((cutting_min_segments_slider*10) != cutting_min_segments)
  {
    cutting_min_segments = (cutting_min_segments_slider*10);
    std::cout << "cutting_min_segments: " << cutting_min_segments << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "use_local_constrain");
  cv::createTrackbar( TrackbarName, "Settings window", &use_local_constrain_slider, use_local_constrain_max, on_trackbar );
  /// Show some stuff
  if(use_local_constrain_slider != use_local_constrain)
  {
    use_local_constrain = use_local_constrain_slider;
    std::cout << "use_local_constrain: " << use_local_constrain << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "use_directed_cutting");
  cv::createTrackbar( TrackbarName, "Settings window", &use_directed_cutting_slider, use_directed_cutting_max, on_trackbar );
  /// Show some stuff
  if(use_directed_cutting_slider != use_directed_cutting)
  {
    use_directed_cutting = use_directed_cutting_slider;
    std::cout << "use_directed_cutting: " << use_directed_cutting << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "use_clean_cutting");
  cv::createTrackbar( TrackbarName, "Settings window", &use_clean_cutting_slider, use_clean_cutting_max, on_trackbar );
  /// Show some stuff
  if(use_clean_cutting_slider != use_clean_cutting)
  {
    use_clean_cutting = use_clean_cutting_slider;
    std::cout << "use_clean_cutting: " << use_clean_cutting << std::endl;
    update = true;
    presentationUpdate = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "ransac_iterations");
  cv::createTrackbar( TrackbarName, "Settings window", &ransac_iterations_slider, ransac_iterations_max, on_trackbar );
  /// Show some stuff
  if((ransac_iterations_slider*1000) != ransac_iterations)
  {
    ransac_iterations = (ransac_iterations_slider*1000);
    std::cout << "ransac_iterations: " << ransac_iterations << std::endl;
    update = true;
    presentationUpdate = true;
  }
}
