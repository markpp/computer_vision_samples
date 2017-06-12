#include "FeatureExtractor.hpp"

FeatureExtractor::FeatureExtractor()
{
}

std::string FeatureExtractor::ZeroPadNumber(int num)
{
    std::ostringstream ss;
    ss << std::setw( 4 ) << std::setfill( '0' ) << num;
    return ss.str();
}

void FeatureExtractor::openOutputStream(int counter)
{
    featureOutputStreamString = std::stringstream();
    featureOutputStreamString << "../output/features/features_" << ZeroPadNumber(counter) << ".txt";
    featurePath = featureOutputStreamString.str();
    std::cout << featurePath << std::endl;
    featureOutputStream.open(featurePath);
}

void FeatureExtractor::endOutputStream()
{
  featureOutputStream.close();
}

void FeatureExtractor::extractSuperVoxelFeatures(pcl::PointCloud <PointTL>::Ptr annoPC, std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxels, std::multimap<uint32_t, uint32_t> &supervoxel_adjacency, int count)
{
  //Because of the voxelgrid used in sv segmentation, some points in the supervoxels are slightly different from the orinal. Find the best match: http://www.pcl-users.org/Find-closest-point-to-a-given-location-td4023326.html

  /* Initialize variable for determining the super voxel label */
  const int MAX_CLASSES = 255;
  std::vector<uint32_t> classCounters(MAX_CLASSES);
  uint32_t label;

  /* Sort through all supervoxels and extract features from the useful ones */
  for (const auto &it : supervoxels)
  {
    uint32_t SupervoxelSize = it.second->voxels_->size();
    /* Ignore small supervoxels without neighbours */
    if(SupervoxelSize > 2 && supervoxel_adjacency.count(it.first) > 0)
    {
      std::vector<float> nX, nY, nZ; // Variables for calculating position mean and std
      std::vector<float> cL, ca, cb; // Variables for calculating color mean and std
      //std::cout << "SV_Num " << it.first << " Size: " << SupervoxelSize << " Center " << it.second->centroid_ << std::endl;
      for(unsigned int i=0; i<SupervoxelSize; i++)
      {
        if(!isnan(it.second->normals_->points[i].normal_x) &&
           !isnan(it.second->normals_->points[i].normal_y) &&
           !isnan(it.second->normals_->points[i].normal_z))
        {
          // Directly copy the position
          nX.push_back(it.second->normals_->points[i].normal_x);
          nY.push_back(it.second->normals_->points[i].normal_y);
          nZ.push_back(it.second->normals_->points[i].normal_z);

          // Convert from RGB to Lab and copy to the
          Eigen::Vector3i colorRGB((int)it.second->voxels_->points[i].r, (int)it.second->voxels_->points[i].b, (int)it.second->voxels_->points[i].g);
          Eigen::Vector3f colorLab = RGB2Lab(colorRGB);
          cL.push_back(colorLab[0]);
          ca.push_back(colorLab[1]);
          cb.push_back(colorLab[2]);

          // Look for a voxel match in the annotated point cloud and vote on supervoxel label
          for(unsigned int j=0; j<annoPC->size(); j++)
          {
            if(it.second->voxels_->points[i].x == annoPC->points[j].x &&
               it.second->voxels_->points[i].y == annoPC->points[j].y &&
               it.second->voxels_->points[i].z == annoPC->points[j].z)
            {
              // Cast label vote for the current voxel
              classCounters[annoPC->points[j].label]++;
            }
          }
        }
        else
        {
          std::cout << "nanana" << std::endl;
        }
      }

      double supervoxelMeanNormalX, supervoxelMeanNormalY, supervoxelMeanNormalZ;
      double supervoxelStddevNormalX, supervoxelStddevNormalY, supervoxelStddevNormalZ;
      pcl::getMeanStdDev(nX, supervoxelMeanNormalX, supervoxelStddevNormalX);
      pcl::getMeanStdDev(nY, supervoxelMeanNormalY, supervoxelStddevNormalY);
      pcl::getMeanStdDev(nZ, supervoxelMeanNormalZ, supervoxelStddevNormalZ);

      double supervoxelMeanColorL, supervoxelMeanColora, supervoxelMeanColorb;
      double supervoxelStddevColorL, supervoxelStddevColora, supervoxelStddevColorb;
      pcl::getMeanStdDev(cL, supervoxelMeanColorL, supervoxelStddevColorL);
      pcl::getMeanStdDev(ca, supervoxelMeanColora, supervoxelStddevColora);
      pcl::getMeanStdDev(cb, supervoxelMeanColorb, supervoxelStddevColorb);

      //
      if(!isnan(supervoxelStddevNormalX) && !isnan(supervoxelStddevNormalY) && !isnan(supervoxelStddevNormalZ))
      {
        uint32_t sum = 0;
        uint32_t winnerLabel = 50; // Default label is 50
        uint32_t winnerVotes = 0;
        for(uint32_t k = 0; k < classCounters.size(); k++)
        {
          sum += classCounters[k];
          if(winnerVotes < classCounters[k])
          {
            winnerLabel = k;
            winnerVotes = classCounters[winnerLabel];
          }
        }
        /*
        std::cout << "Winner: " << winnerLabel << "!, "
                  << sum << "/" << SupervoxelSize << ", "
                  << 50 << " : " << classCounters[50] << ", "
                  << 100 << " : " << classCounters[100] << ", "
                  << 150 << " : " << classCounters[150] << ", "
                  << 200 << " : " << classCounters[200] << std::endl;
        */
        // Reset votes
        classCounters[50] = 0;
        classCounters[100] = 0;
        classCounters[150] = 0;
        classCounters[200] = 0;

        /* Create stringstream of neighbour ids */
        std::stringstream neighboursStream;
        neighboursStream.clear();
        const auto it_first = supervoxel_adjacency.lower_bound(it.first);
        const auto it_afterLast = supervoxel_adjacency.upper_bound(it.first);

        bool firstEntry = true;
        for (auto it_neighbour = it_first ; it_neighbour != it_afterLast; ++it_neighbour)
        {
          if(!firstEntry){neighboursStream << ",";}
          firstEntry = false;
          neighboursStream << it_neighbour->second - 1; // Ids are shifted by -1 to begin from 0
        }

        // Convert and copy the supervoxel centroid color from RGB to Lab
        Eigen::Vector3i colorRGB((int)it.second->centroid_.r, (int)it.second->centroid_.g, (int)it.second->centroid_.b);
        Eigen::Vector3f colorLab = RGB2Lab(colorRGB);

        // Compute eigenvalues for the supervoxel
        pcl::PCA<pcl::PointXYZRGB> pca;
        pca.setInputCloud(it.second->voxels_);
        Eigen::Vector3f eigenVals = pca.getEigenValues();
        //std::cout << "1st eigen: " << eigenVals[0] << " 2nd eigen: " << eigenVals[1] << " 3rd eigen: " << eigenVals[2] << std::endl;

        /* JSON encode the super voxel features */
        //https://github.com/nlohmann/json
        nlohmann::json out = {
          {"label", winnerLabel},
          {"classification", "NA"},
          {"sampleID", count},
          {"supervoxID", it.first - 1}, // Ids are shifted by -1 to begin from 0
          {"neighbours", {
            //{"count", supervoxel_adjacency.count(it.first)},
            {"IDs", neighboursStream.str()},
          }},
          {"size", SupervoxelSize},
          {"features", {
            {"position", {
              {"x", it.second->centroid_.x},
              {"y", it.second->centroid_.y},
              {"z", it.second->centroid_.z},
            }},
            {"RGB", {
              {"r", (int)it.second->centroid_.r},
              {"g", (int)it.second->centroid_.g},
              {"b", (int)it.second->centroid_.b},
            }},
            {"LAB", {
              {"L", (int)colorLab[0]},
              {"a", (int)colorLab[1]},
              {"b", (int)colorLab[2]},
            }},
            {"normal", {
              {"x", (int)(((it.second->normal_.normal_x+1)/2)*255)},
              {"y", (int)(((it.second->normal_.normal_y+1)/2)*255)},
              {"z", (int)((((it.second->normal_.normal_z*-1)+1)/2)*255)},
            }},
            //{"stdDevNormal", {
            //  {"x", supervoxelStddevNormalX},
            //  {"y", supervoxelStddevNormalY},
            //  {"z", supervoxelStddevNormalZ},
            //}},
            //{"meanLAB", {
            //  {"L", (int)supervoxelMeanColorL},
            //  {"a", (int)supervoxelMeanColora},
            //  {"b", (int)supervoxelMeanColorb},
            //}},
            {"stdDevLAB", {
              {"L", (int)supervoxelStddevColorL},
              {"a", (int)supervoxelStddevColora},
              {"b", (int)supervoxelStddevColorb},
            }},
            //{"eigen", {
            //  {"e0", eigenVals[0]},
            //  {"e1", eigenVals[1]},
            //  {"e2", eigenVals[2]},
            //}},
          }},
        };
        featureOutputStream << out << std::endl;
      }
    }
    /*
    else
    {
      //std::cout << "bad: " << it.first << std::endl;
      Eigen::Vector3i colorRGB((int)it.second->centroid_.r, (int)it.second->centroid_.g, (int)it.second->centroid_.b);
      Eigen::Vector3f colorLab = RGB2Lab(colorRGB);

      nlohmann::json out = {
        {"label", 50},
        {"classification", "NA"},
        {"sampleID", count},
        {"supervoxID", it.first-1},
        {"neighbours", {
          {"IDs", ""}, // always empty
        }},
        {"size", SupervoxelSize},
        {"features", {
          {"position", {
            {"x", it.second->centroid_.x},
            {"y", it.second->centroid_.y},
            {"z", it.second->centroid_.z},
          }},
          {"RGB", {
            {"r", (int)it.second->centroid_.r},
            {"g", (int)it.second->centroid_.g},
            {"b", (int)it.second->centroid_.b},
          }},
          {"LAB", {
            {"L", (int)colorLab[0]},
            {"a", (int)colorLab[1]},
            {"b", (int)colorLab[2]},
          }},
          {"normal", {
            {"x", (int)(0)},
            {"y", (int)(0)},
            {"z", (int)(0)},
          }},
          {"stdDevLAB", {
            {"L", (int)(0)},
            {"a", (int)(0)},
            {"b", (int)(0)},
          }},
        }},
      };
    }
    */
  }
}

Eigen::Vector3f FeatureExtractor::RGB2Lab (const Eigen::Vector3i& colorRGB)
{
  // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
  // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

  double R, G, B, X, Y, Z;

  // map sRGB values to [0, 1]
  R = colorRGB[0] / 255.0;
  G = colorRGB[1] / 255.0;
  B = colorRGB[2] / 255.0;

  // linearize sRGB values
  if (R > 0.04045)
    R = pow ( (R + 0.055) / 1.055, 2.4);
  else
    R = R / 12.92;

  if (G > 0.04045)
    G = pow ( (G + 0.055) / 1.055, 2.4);
  else
    G = G / 12.92;

  if (B > 0.04045)
    B = pow ( (B + 0.055) / 1.055, 2.4);
  else
    B = B / 12.92;

  // postponed:
  //    R *= 100.0;
  //    G *= 100.0;
  //    B *= 100.0;

  // linear sRGB -> CIEXYZ
  X = R * 0.4124 + G * 0.3576 + B * 0.1805;
  Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
  Z = R * 0.0193 + G * 0.1192 + B * 0.9505;

  // *= 100.0 including:
  X /= 0.95047;  //95.047;
  //    Y /= 1;//100.000;
  Z /= 1.08883;  //108.883;

  // CIEXYZ -> CIELAB
  if (X > 0.008856)
    X = pow (X, 1.0 / 3.0);
  else
    X = 7.787 * X + 16.0 / 116.0;

  if (Y > 0.008856)
    Y = pow (Y, 1.0 / 3.0);
  else
    Y = 7.787 * Y + 16.0 / 116.0;

  if (Z > 0.008856)
    Z = pow (Z, 1.0 / 3.0);
  else
    Z = 7.787 * Z + 16.0 / 116.0;

  Eigen::Vector3f colorLab;
  colorLab[0] = static_cast<float> (116.0 * Y - 16.0);
  colorLab[1] = static_cast<float> (500.0 * (X - Y));
  colorLab[2] = static_cast<float> (200.0 * (Y - Z));

  return colorLab;
}

void FeatureExtractor::extractVoxelFeatures(pcl::PointCloud <PointTL>::Ptr annoPC, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures_ptr)
{
  //For unknown reasons some points in the supervoxels are slightly different from the orinal. Find the best match: http://www.pcl-users.org/Find-closest-point-to-a-given-location-td4023326.html
/*
for (const auto &it : supervoxels)
{
  featureOutputStream << it.first << std::endl;
  for(unsigned int i=0; i<it.second->voxels_->size(); i++)
  {
      featureOutputStream << it.second->voxels_->points[i].x << "," << it.second->voxels_->points[i].y << "," << it.second->voxels_->points[i].z << std::endl;
  }
}
*/

  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RGBAcloud_ptr;
  //pcl::copyPointCloud(*annoPC, *RGBAcloud_ptr);
  pcl::PointCloud<pcl::PointXYZLAB> LABcloud;
  convertRGBAToLAB (annoPC, LABcloud);
  for(unsigned int i=0; i<annoPC->size(); i++)
  {
    if(!isnan(normals->points[i].normal_x) && !isnan(normals->points[i].normal_y) && !isnan(normals->points[i].normal_z) &&
       !isnan(principalCurvatures_ptr->points[i].principal_curvature_x) && !isnan(principalCurvatures_ptr->points[i].principal_curvature_y) && !isnan(principalCurvatures_ptr->points[i].principal_curvature_z) &&
       !isnan(principalCurvatures_ptr->points[i].pc1) && !isnan(principalCurvatures_ptr->points[i].pc2))
    {
      //std::cout << it.second->normals_->points[i].normal_z << std::endl;

      //nX.push_back(it.second->normals_->points[i].normal_x);
      //nY.push_back(it.second->normals_->points[i].normal_y);
      //nZ.push_back(it.second->normals_->points[i].normal_z);
      /*
      for(unsigned int j=0; j<annoPC->size(); j++)
      {
        if(    colorPC->points[i].x == annoPC->points[j].x
            && colorPC->points[i].y == annoPC->points[j].y
            && colorPC->points[i].z == annoPC->points[j].z)
        {
          //std::cout << "point " << i << ": " << it.voxels_->points[i] << std::endl;

          classCounters[annoPC->points[j].label]++;
        }
      }
      */
      //https://github.com/nlohmann/json
      nlohmann::json out = {
        {"label", annoPC->points[i].label},
        {"classification", "NA"},
        {"ID", i},
        {"size", 1},
        //{"featureVector", {SupervoxelSize, it.second->centroid_.x, it.second->centroid_.y, it.second->centroid_.z, (int)it.second->centroid_.r, (int)it.second->centroid_.g, (int)it.second->centroid_.b, (int)(((it.second->normal_.normal_x+1)/2)*255), (int)(((it.second->normal_.normal_y+1)/2)*255), (int)((((it.second->normal_.normal_z*-1)+1)/2)*255)}},
        {"features", {
          {"position", {
            {"x", annoPC->points[i].x},
            {"y", annoPC->points[i].y},
            {"z", annoPC->points[i].z},
          }},
          {"RGB", {
            {"r", (int)annoPC->points[i].r},
            {"g", (int)annoPC->points[i].g},
            {"b", (int)annoPC->points[i].b},
          }},
          {"LAB", {
            {"L", (int)LABcloud.points[i].L},
            {"a", (int)LABcloud.points[i].a},
            {"b", (int)LABcloud.points[i].b},
          }},
          {"normal", {
            {"x", (int)(((normals->points[i].normal_x+1)/2)*255)},
            {"y", (int)(((normals->points[i].normal_y+1)/2)*255)},
            {"z", (int)((((normals->points[i].normal_z*-1)+1)/2)*255)},
          }},
          {"normal1", {
            {"x", normals->points[i].normal_x},
            {"y", normals->points[i].normal_y},
            {"z", normals->points[i].normal_z*-1},
          }},
          {"curve", {
            {"x", principalCurvatures_ptr->points[i].principal_curvature_x},
            {"y", principalCurvatures_ptr->points[i].principal_curvature_y},
            {"z", principalCurvatures_ptr->points[i].principal_curvature_z},
            {"pc1", principalCurvatures_ptr->points[i].pc1},
            {"pc2", principalCurvatures_ptr->points[i].pc2},
          }},
          //{"stdDevNormal", {
          //  {"x", supervoxelStddevNormalX},
          //  {"y", supervoxelStddevNormalY},
          //  {"z", supervoxelStddevNormalZ},
          //}},
        }},
      };
      featureOutputStream << out << std::endl;
    }


  }
  //std::cout << "3 Mean: x " << it.second->normal_.normal_x << "," << it.second->normal_.normal_y << "," << it.second->normal_.normal_z << std::endl << std::endl;

  /*
  featureOutputStream << winnerLabel << "," << SupervoxelSize << "," << it.second->centroid_.y << ","
                      << (int)it.second->centroid_.r << "," << (int)it.second->centroid_.g << ","
                      << (int)it.second->centroid_.b << "," << (int)(((it.second->normal_.normal_x+1)/2)*255)
                      << "," << (int)(((it.second->normal_.normal_y+1)/2)*255) << ","
                      << (int)((((it.second->normal_.normal_z*-1)+1)/2)*255) << ","
                      << supervoxelStddevNormalX << "," << supervoxelStddevNormalY << ","
                      << supervoxelStddevNormalZ << std::endl;
  */


}

// convert a PointXYZRGBA cloud to a PointXYZLAB cloud
void FeatureExtractor::convertRGBAToLAB (pcl::PointCloud <PointTL>::Ptr in, pcl::PointCloud<pcl::PointXYZLAB>& out)
{
  out.resize (in->size ());

  for (size_t i = 0; i < in->size (); ++i)
  {
    out[i].x = in->points[i].x;
    out[i].y = in->points[i].y;
    out[i].z = in->points[i].z;
    out[i].data[3] = 1.0;  // important for homogeneous coordinates

    Eigen::Vector3f lab = RGB2Lab (in->points[i].getRGBVector3i ());
    out[i].L = lab[0];
    out[i].a = lab[1];
    out[i].b = lab[2];
  }
}
