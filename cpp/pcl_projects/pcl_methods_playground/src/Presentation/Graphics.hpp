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
  boost::shared_ptr<pcl::visualization::PCLVisualizer> initViewer(float pos_x,
                                                                    float pos_y,
                                                                    float pos_z,
                                                                    float view_x,
                                                                    float view_y,
                                                                    float view_z,
                                                                    float up_x,
                                                                    float up_y,
                                                                    float up_z);
  void drawBoundary_z(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float, float);
  void drawBoundary_y(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float, float);
  void removeBoundary_z(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
  void removeBoundary_y(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);



private:



};
