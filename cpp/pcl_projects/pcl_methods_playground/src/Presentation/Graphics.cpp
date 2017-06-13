#include "Graphics.hpp"

Graphics::Graphics(){}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Graphics::initViewer(float pos_x,
                                                                          float pos_y,
                                                                          float pos_z,
                                                                          float view_x,
                                                                          float view_y,
                                                                          float view_z,
                                                                          float up_x,
                                                                          float up_y,
                                                                          float up_z)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (255, 255, 255);
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  //x=red axis, y=green axis, z=blue
  viewer->addCoordinateSystem (0.05, 0.0, 0.0, 0.0);
  viewer->initCameraParameters ();

  viewer->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z);

  std::vector<pcl::visualization::Camera> cam;
  //Save the position of the camera
  viewer->getCameras(cam);
  //Print recorded points on the screen:
  cout << "Cam: " << endl
       << " - pos: ("   << cam[0].pos[0]   << ", "  << cam[0].pos[1]   << ", "  << cam[0].pos[2]   << ")" << endl
       << " - view: ("  << cam[0].view[0]  << ", "  << cam[0].view[1]  << ", "  << cam[0].view[2]  << ")" << endl
       << " - focal: (" << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")" << endl;

  return (viewer);
}

void Graphics::drawBoundary_z(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float lineLength, float zmax)
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
  viewer->addPlane (coeffs, "z_plane");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "z_plane");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "z_plane");

}

void Graphics::drawBoundary_y(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float lineLength, float ymax)
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
  coeffs.values.push_back(1.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(ymax);
  viewer->addPlane (coeffs, "y_plane");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "y_plane");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "y_plane");

}

void Graphics::removeBoundary_z(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  /*
  viewer->removeShape ("ld1", 0);
  viewer->removeShape ("ld2", 0);
  viewer->removeShape ("lt1", 0);
  viewer->removeShape ("lb2", 0);
  viewer->removeShape ("ll1", 0);
  viewer->removeShape ("lr1", 0);
  */
  viewer->removeShape ("z_plane", 0);
}

void Graphics::removeBoundary_y(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  /*
  viewer->removeShape ("ld1", 0);
  viewer->removeShape ("ld2", 0);
  viewer->removeShape ("lt1", 0);
  viewer->removeShape ("lb2", 0);
  viewer->removeShape ("ll1", 0);
  viewer->removeShape ("lr1", 0);
  */
  viewer->removeShape ("y_plane", 0);
}
