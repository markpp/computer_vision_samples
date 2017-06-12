#pragma once
/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <pcl/point_types.h>

// Types
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBA PointTA;
//typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointXYZRGBL PointTL;
typedef pcl::PointXYZL PointL;

enum modes {PassThroughMode, smoothingMode, NormalsMode, CRGSMode, RGSMode, colorDepthMode, SVCMode, LCCPMode, CPCMode, NUMBER_OF_MODE};
#define numModes NUMBER_OF_MODE
