#include "SVC.hpp"

SVC::SVC()
{
}

void SVC::segmentPC(pcl::PointCloud <PointT>::Ptr inputPC, float voxel_resolution, float seed_resolution, float color_importance, float spatial_importance, float normal_importance, int refinement_itr)
{
  /* Initialize SupervoxelClustering object */
  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  super.setInputCloud (inputPC);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  /* Extracting supervoxels */
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
  //pcl::console::print_info ("id at last sv\n", supervoxel_clusters.at(supervoxel_clusters.size()) );

  /* Prepare some of the different visualizations */
  voxel_centroid_cloud = super.getVoxelCentroidCloud (); //
  labeled_voxel_cloud = super.getLabeledVoxelCloud (); // Points belonging to the same supervoxel have the same label
  voxel_colored_cloud = super.getLabeledCloud (); //

  /* Refine super segmentation */
  super.refineSupervoxels(refinement_itr, supervoxel_clusters_refined);
  voxel_colored_cloud_refined = super.getLabeledCloud ();

  /* Calculate the normal for each supervoxel cluster */
  sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);

  /* Calculate supervoxel adjacency map */
  super.getSupervoxelAdjacency (supervoxel_adjacency);
}

void SVC::fixIds()
{
  /* Fix holes in ids for supervoxel clusters */
  std::map <uint32_t, uint32_t> replacementMap;
  int count = 1;
  for (const auto &it : supervoxel_clusters)
  {
    if(count != it.first)
    {
      supervoxel_clusters_fixed.insert(std::pair<uint32_t, pcl::Supervoxel<PointT>::Ptr>(count, it.second));
      replacementMap.insert(std::pair<uint32_t, uint32_t>(it.first, count));
    }
    else
    {
      supervoxel_clusters_fixed.insert(std::pair<uint32_t, pcl::Supervoxel<PointT>::Ptr>(it.first, it.second));
    }
    count++;
    std::cout << it.first << std::endl;
  }

  /*
  // Debug sv id map
  for (const auto &it : replacementMap)
  {
      std::cout << "replacementMap: 1: " << it.first << " 2: " << it.second <<  std::endl;
  }
  */

  /* Fix holes in ids for adjacency map */
  //look for neighbour ids that must be changed
  for (auto &it_adj : supervoxel_adjacency)
  {
    // Keep as is
    int first = it_adj.first;
    int second = it_adj.second;
    //look if replacement mapping is nessesary
    for (auto &it_repmap : replacementMap)
    {
      // if a neighbour id is in the map, change it to the remap value
      if(it_adj.second == it_repmap.first)
      {
        //std::cout << "it_adj.first: " << it_adj.first << " it_repmap.first: " << it_repmap.second << std::endl;
        second = it_repmap.second;
      }
      if(it_adj.first == it_repmap.first)
      {
        //std::cout << "it_adj.first: " << it_adj.first << " it_repmap.first: " << it_repmap.second << std::endl;
        first = it_repmap.second;
      }
    }
    supervoxel_adjacency_fixed.insert(std::pair<uint32_t, uint32_t>(first, second));
  }
  /*
  // Debug id assignment map
  for (const auto &it : supervoxel_adjacency_fixed)
  {
      std::cout << "1: " << it.first << "2: " << it.second <<  std::endl;
  }
  */
}

void SVC::drawAdjacencyGraph(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
  for ( ; label_itr != supervoxel_adjacency.end (); )
  {
    //First get the label
    uint32_t supervoxel_label = label_itr->first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    pcl::PointCloud<PointTA> adjacent_supervoxel_centers;
    std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
    for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
    {
      //std::cout << "her2: " << adjacent_itr->second << std::endl;
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
      adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }
    //Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    //addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

    //Iterate through all adjacent points, and add a center point to adjacent point pair
    pcl::PointCloud<PointTA>::iterator adjacent_itr2 = adjacent_supervoxel_centers.begin ();
    for ( ; adjacent_itr2 != adjacent_supervoxel_centers.end (); ++adjacent_itr2)
    {
      points->InsertNextPoint (supervoxel->centroid_.data);
      points->InsertNextPoint (adjacent_itr2->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    // Add the points to the dataset
    polyData->SetPoints (points);
    polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
    for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
      polyLine->GetPointIds ()->SetId (i,i);
    cells->InsertNextCell (polyLine);
    // Add the lines to the dataset
    polyData->SetLines (cells);

    if(!viewer->contains(ss.str()))
    {
      viewer->addModelFromPolyData (polyData,ss.str());
    }
    polyIDs.push_back(ss.str());
    //polyCounter = supervoxel_adjacency.end ();
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }
}

void SVC::showSuperVoxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer, uint32_t maxNumberOfSupervoxels)
{
  for(auto it = supervoxel_clusters.begin(); it != supervoxel_clusters.end(); it++)
  {
    if(it->first < maxNumberOfSupervoxels)
    {
      std::stringstream ss;
      ss << "SV_" << it->first;
      supervoxelIDs.push_back(ss.str());
      if (!viewer->updatePointCloud(it->second->voxels_, ss.str()))
      {
        viewer->addPointCloud (it->second->voxels_, ss.str());
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, ss.str());
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, ss.str());
      }
    }
  }
}

void SVC::clearOldPolygons(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  for(int i = 0; i<polyIDs.size(); i++)
  {
    //viewer->removeAllShapes();
    if(viewer->contains(polyIDs.at(i)))
    {
      viewer->removeShape(polyIDs.at(i));
    }
  }
  polyIDs.clear();
}

void SVC::clearOldSupervoxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  for(int i = 0; i<supervoxelIDs.size(); i++)
  {
    //viewer->removeAllShapes();
    if(viewer->contains(supervoxelIDs.at(i)))
    {
      viewer->removePointCloud(supervoxelIDs.at(i));
    }
  }
  supervoxelIDs.clear();
}
