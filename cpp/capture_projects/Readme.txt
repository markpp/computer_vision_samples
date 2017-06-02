Code for capturing point clouds using real sense, ZED and Kinect V2.

The clean.zip in “PCLChicken/Capture/PCLRealSenseGrabber/build” contains a fresh folder structure for output files.


Kinect 2 projects:

This captures everything we need except ir, might also need to untoggle capture mode after first frame.
https://github.com/nznobody/KinectGrabber/blob/Kinect2Grabber/Sample/kinect2_grabber.h
 
This captures the point cloud
https://github.com/UnaNancyOwen/Kinect2Sample/tree/master/sample/PointCloud
 
this captures depth and color, could be extended with ir and with a projection to point cloud
https://github.com/markpp/kinect2grabber/blob/master/src/Kinect2Manager.cpp
 
