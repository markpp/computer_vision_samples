#include "PrincipalCurves.hpp"

PrincipalCurves::PrincipalCurves()
{

}

/*
"Perform Principal Components Analysis (PCA) on the point normals of a surface patch in the tangent plane of the given
point normal, and return the principal curvature (eigenvector of the max eigenvalue), along with both the max (pc1) and
min (pc2) eigenvalues."

This should be extended a little bit, but basically what we're doing there is, for each point:

  // construct a projection matrix for the tangent plane given by the normal of the query point
  // project all normals from a k-neighborhood (surface patch) onto the tangent plane
  // compute a centroid in that projected space, and a covariance matrix
  // perform eigen decomposition to obtain the "principal directions"

Therefore the values obtained in the end, are nothing else but:

100   pcl::eigen33 (covariance_matrix_, eigenvectors_, eigenvalues_);
101
102   pcx = eigenvectors_ (0, 2);
103   pcy = eigenvectors_ (1, 2);
104   pcz = eigenvectors_ (2, 2);
105   pc1 = eigenvalues_ (2);
106   pc2 = eigenvalues_ (1);
*/

void PrincipalCurves::generatePrincipalCurves(pcl::PointCloud <pcl::PointXYZ>::Ptr inputPC_ptr)
{

  // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud (inputPC_ptr);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

    normalEstimation.setRadiusSearch (0.03);

    normalEstimation.compute (*cloudWithNormals);

    // Setup the principal curvatures computation
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    // Provide the original point cloud (without normals)
    principalCurvaturesEstimation.setInputCloud (inputPC_ptr);

    // Provide the point cloud with normals
    principalCurvaturesEstimation.setInputNormals (cloudWithNormals);
    //pcl::io::savePLYFileASCII("norm.ply", *cloudWithNormals);
    // Use the same KdTree from the normal estimation
    principalCurvaturesEstimation.setSearchMethod (tree);
    principalCurvaturesEstimation.setRadiusSearch (1.0);

    // Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principalCurvaturesEstimation.compute (*principalCurvatures);

    std::cout << "output points.size (): " << principalCurvatures->points.size () << std::endl;

    //pcl::io::savePLYFileASCII("hey.ply", *principalCurvatures);
    cloudWithNormals_ptr = cloudWithNormals;
  principalCurvatures_ptr = principalCurvatures;
}
