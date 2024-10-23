#pragma once

// Third Party
#include "icp/types.hpp"
#include "icp/icp_cloud.hpp"
#include <iostream>


namespace ICP {
  struct ICPResults {
    Mat4 trans;
    double error;
    int iters;
    bool converged;
  };

  std::ostream& operator<< (std::ostream &out, const ICPResults &results);

namespace Utils {

  // Computes the rotation matrix between two pointclouds via singular value decomposition
  // of the cross-covariance of src and ref.
  // src and ref must contain the same number of points.
  // The result will only make sense if the two pointclouds have been centered
  // (shifted by their centroid) such that they overlap.
  Mat3 ComputeRotationMatrix(const PointCloudMat& src, const PointCloudMat& ref);

  // Computes the 4d homogeneous matrix encoding rotation + translation between two pointclouds
  Mat4 ComputeBestFitTransform(ICPPointCloud &src, ICPPointCloud &ref);

  // The actual ICP business logic.
  // given a source pointcloud and a reference pointcloud, this function
  // - finds closest points in the reference cloud to each point in the source cloud
  // - estimates a transform between the two pointclouds
  // - transforms the source by the estimated transform
  // until either the max iterations have been reached,
  // or the mean distances between closest points are minimized below a convergence threshold.
  // Returns the transform between the original src and modified src
  ICPResults MatchSourceScanToReferenceScan(ICPPointCloud& src, ICPPointCloud& ref, int max_iterations, double tolerance);
} // namespace Utils
} // namespace ICP
