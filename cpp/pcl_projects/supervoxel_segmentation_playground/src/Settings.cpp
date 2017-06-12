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

bool Settings::setVariables()
{
  /// Create Sliders
  setSVCSettings();

  if(SVCModeSwitch == SUPERVOXEL)
  {
    setSVCNumberOfSupervoxels();
  }

  setNormalPressentationSettings();

  return settingsUpdated;
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
    settingsUpdated = true;
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
    settingsUpdated = true;
    presentationUpdated = true;
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
    settingsUpdated = true;
    presentationUpdated = true;
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
    settingsUpdated = true;
    presentationUpdated = true;
  }

  /// Create Trackbar
  sprintf( TrackbarName, "SVC AdjacencyGraphSwitch");
  cv::createTrackbar( TrackbarName, "Settings window", &AdjacencyGraphSwitch_slider, AdjacencyGraphSwitch_slider_max, on_trackbar );
  /// Show some stuff
  if(AdjacencyGraphSwitch_slider != AdjacencyGraphSwitch)
  {
    AdjacencyGraphSwitch = AdjacencyGraphSwitch_slider;
    std::cout << "AdjacencyGraphSwitch: " << AdjacencyGraphSwitch << std::endl;
    settingsUpdated = true;
    presentationUpdated = true;
  }

  /// Create Trackbar
  sprintf( TrackbarName, "SVC voxel_resolution");
  cv::createTrackbar( TrackbarName, "Settings window", &voxel_resolution_slider, voxel_resolution_max, on_trackbar );
  /// Show some stuff
  if(((float)voxel_resolution_slider/1000) != voxel_resolution)
  {
    voxel_resolution = ((float)voxel_resolution_slider/1000);
    std::cout << "voxel_resolution: " << voxel_resolution << std::endl;
    settingsUpdated = true;
    presentationUpdated = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "SVC seed_resolution");
  cv::createTrackbar( TrackbarName, "Settings window", &seed_resolution_slider, seed_resolution_max, on_trackbar );
  /// Show some stuff
  if(((float)seed_resolution_slider/1000) != seed_resolution)
  {
    seed_resolution = ((float)seed_resolution_slider/1000);
    std::cout << "seed_resolution: " << seed_resolution << std::endl;
    settingsUpdated = true;
    presentationUpdated = true;
    resetWindow();
  }
  /// Create Trackbar
  sprintf( TrackbarName, "SVC color_importance");
  cv::createTrackbar( TrackbarName, "Settings window", &color_importance_slider, color_importance_max, on_trackbar );
  /// Show some stuff
  if(((float)color_importance_slider/10) != color_importance)
  {
    color_importance = ((float)color_importance_slider/10);
    std::cout << "color_importance: " << color_importance << std::endl;
    settingsUpdated = true;
    presentationUpdated = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "SVC spatial_importance");
  cv::createTrackbar( TrackbarName, "Settings window", &spatial_importance_slider, spatial_importance_max, on_trackbar );
  /// Show some stuff
  if(((float)spatial_importance_slider/10) != spatial_importance)
  {
    spatial_importance = ((float)spatial_importance_slider/10);
    std::cout << "spatial_importance: " << spatial_importance << std::endl;
    settingsUpdated = true;
    presentationUpdated = true;
  }
  /// Create Trackbar
  sprintf( TrackbarName, "SVC normal_importance");
  cv::createTrackbar( TrackbarName, "Settings window", &normal_importance_slider, normal_importance_max, on_trackbar );
  /// Show some stuff
  if(((float)normal_importance_slider/10) != normal_importance)
  {
    normal_importance = ((float)normal_importance_slider/10);
    std::cout << "normal_importance: " << normal_importance << std::endl;
    settingsUpdated = true;
    presentationUpdated = true;
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
      settingsUpdated = true;
      presentationUpdated = true;
    }
  }
}

void Settings::setSVCNumberOfSupervoxels()
{
  /// Create Trackbar
  sprintf( TrackbarName, "SVC ShowUptoVoxels");
  cv::createTrackbar( TrackbarName, "Settings window", &ShowUptoVoxels_slider, ShowUptoVoxels_max, on_trackbar );
  /// Show some stuff
  if(ShowUptoVoxels_slider != ShowUptoVoxels)
  {
    ShowUptoVoxels = ShowUptoVoxels_slider;
    std::cout << "ShowUptoVoxels: " << ShowUptoVoxels << std::endl;
    settingsUpdated = true;
    presentationUpdated = true;
  }
}
