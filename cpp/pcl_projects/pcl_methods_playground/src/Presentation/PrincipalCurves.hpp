/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/io/ply_io.h>

class PrincipalCurves {
public:
  PrincipalCurves();

  void generatePrincipalCurves(pcl::PointCloud <pcl::PointXYZ>::Ptr);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals_ptr; 

private:


};
