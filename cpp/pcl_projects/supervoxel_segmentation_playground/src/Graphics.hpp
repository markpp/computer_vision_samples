/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <boost/thread/thread.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class Graphics {
public:
  Graphics();
  boost::shared_ptr<pcl::visualization::PCLVisualizer> initViewer();
  void drawBoundary(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float, float);
  void removeBoundary(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);


private:



};
