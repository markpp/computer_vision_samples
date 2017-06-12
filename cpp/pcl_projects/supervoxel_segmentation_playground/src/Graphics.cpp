#include "Graphics.hpp"

Graphics::Graphics()
{
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Graphics::initViewer()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  //viewer->addCoordinateSystem (0.01);
  viewer->initCameraParameters ();

  // adding sphere, useful for seeing what the viewer i centering on
  pcl::PointXYZ FocusPoint;
  FocusPoint.x=0.0; FocusPoint.y=0.0; FocusPoint.z=0.4;
  //viewer->addSphere (FocusPoint,0.01,"sp");
  viewer->setCameraPosition(0.0, 0.0, -0.2, 0.0, 0.0, FocusPoint.z, 0.0, 1.0, 0.0); // cam in z = -0.2, focus on z = 0.2
  return (viewer);
}

void Graphics::drawBoundary(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float lineLength, float zmax)
{
  /*
  viewer->addLine(pcl::PointXYZ(-lineLength,-lineLength,zmax), pcl::PointXYZ(lineLength,lineLength,zmax), 1.0, 0.0, 0.0, "ld1");
  viewer->addLine(pcl::PointXYZ(-lineLength,lineLength,zmax), pcl::PointXYZ(lineLength,-lineLength,zmax), 1.0, 0.0, 0.0, "ld2");
  viewer->addLine(pcl::PointXYZ(-lineLength,-lineLength,zmax), pcl::PointXYZ(lineLength,-lineLength,zmax), 1.0, 0.0, 0.0, "lt1");
  viewer->addLine(pcl::PointXYZ(lineLength,lineLength,zmax), pcl::PointXYZ(lineLength,-lineLength,zmax), 1.0, 0.0, 0.0, "lb2");
  viewer->addLine(pcl::PointXYZ(-lineLength,-lineLength,zmax), pcl::PointXYZ(-lineLength,lineLength,zmax), 1.0, 0.0, 0.0, "ll1");
  viewer->addLine(pcl::PointXYZ(lineLength,lineLength,zmax), pcl::PointXYZ(-lineLength,lineLength,zmax), 1.0, 0.0, 0.0, "lr1");
  */
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(-1.0);
  coeffs.values.push_back(zmax);
  viewer->addPlane (coeffs, "plane");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.25, "plane");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "plane");

}

void Graphics::removeBoundary(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  /*
  viewer->removeShape ("ld1", 0);
  viewer->removeShape ("ld2", 0);
  viewer->removeShape ("lt1", 0);
  viewer->removeShape ("lb2", 0);
  viewer->removeShape ("ll1", 0);
  viewer->removeShape ("lr1", 0);
  */
  viewer->removeShape ("plane", 0);
}
