/*--------- INCLUDES------------*/
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <opencv2/opencv.hpp>

#include "json.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/common/pca.h>

#include "../defines.hpp"

namespace pcl
{
  struct EIGEN_ALIGN16 _PointXYZLAB
  {
    PCL_ADD_POINT4D; // this adds the members x,y,z
    union
    {
      struct
      {
        float L;
        float a;
        float b;
      };
      float data_lab[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** \brief A custom point type for position and CIELAB color value */
  struct PointXYZLAB : public _PointXYZLAB
  {
    inline PointXYZLAB ()
    {
      x = y = z = 0.0f; data[3]     = 1.0f;  // important for homogeneous coordinates
      L = a = b = 0.0f; data_lab[3] = 0.0f;
    }
  };
}

class FeatureExtractor {
public:
  FeatureExtractor();
  void openOutputStream(int);
  void endOutputStream();
  void extractVoxelFeatures(pcl::PointCloud <PointTL>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr);
  void extractSuperVoxelFeatures(pcl::PointCloud <PointTL>::Ptr, std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >, std::multimap<uint32_t, uint32_t> &, int);


private:
  std::string featurePath;
  std::stringstream featureOutputStreamString;
  ofstream featureOutputStream;

  std::string ZeroPadNumber(int);

  Eigen::Vector3f RGB2Lab (const Eigen::Vector3i&);
  void convertRGBAToLAB (pcl::PointCloud <PointTL>::Ptr, pcl::PointCloud<pcl::PointXYZLAB>&);


};
