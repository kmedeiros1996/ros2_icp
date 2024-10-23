
// ICP
#include "icp/utils.hpp"
#include "icp/icp_cloud.hpp"
#include "icp/kd_tree.hpp"

#include <assert.h>
#include <numeric>
namespace ICP {

   std::ostream& operator<< (std::ostream &out, const ICPResults &results) {
    out << "ICP Results:"<<endl;
    out << results.trans<<endl;
    std::string conv_msg = (results.converged) ? "Converged after " : "Not converged after ";
    out << conv_msg << results.iters<<" iterations. ";
    out << "mean error: "<<results.error<<"." << endl;

    return out;
  }

namespace Utils {

Mat3 ComputeRotationMatrix(const PointCloudMat& src, const PointCloudMat& ref) {

  assert(ref.rows() == src.rows() && ref.cols() == src.cols());

  Mat3 cov =  ref.transpose() * src;
  Mat3 rotation_matrix;

  Eigen::JacobiSVD<Mat3> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::MatrixXd u_transpose = svd.matrixU().transpose();
  Eigen::MatrixXd S = svd.singularValues();
  Eigen::MatrixXd V = svd.matrixV();

  rotation_matrix = V*u_transpose;

  // If the rotation matrix has zero determinant, we need to negate the sin/cos values
  if (rotation_matrix.determinant() < 0.0) {
    V.block<1,3>(2,0) *=-1;
    rotation_matrix = V*u_transpose;
  }

  return rotation_matrix;
}

Mat4 ComputeBestFitTransform(ICPPointCloud& src, ICPPointCloud& ref) {  
  Mat4 transformation_matrix = Eigen::MatrixXd::Identity(4,4);
  ICPPointCloud centered_src = src.Centered();
  ICPPointCloud centered_ref = ref.Centered();

  Mat3 rotation_mat = ComputeRotationMatrix(centered_src.Points(), centered_ref.Points());
  Vec3 translation_vec = ref.Centroid().transpose() - rotation_mat * src.Centroid().transpose();

  transformation_matrix.block<3,3>(0,0) = rotation_mat;
  transformation_matrix.block<3,1>(0,3) = translation_vec;

  return transformation_matrix;
}

ICPResults MatchSourceScanToReferenceScan(ICPPointCloud& src, ICPPointCloud& ref, int max_iterations, double tolerance) {
  double prev_error{0.0};
  double mean_error{0.0};
  double diff;

  ICPResults results;

  results.trans = Mat4::Identity();
  results.iters = 0;
  results.error = 0;

  ICPPointCloud current_src(src);

  KDTree ref_tree = ref.ToKDTree();
  for (results.iters = 1; results.iters <= max_iterations; results.iters++) {

    NearestNeighborsResults nns = ref_tree.GetNearestNeighborsResults(current_src.Points());
    ICPPointCloud closest_src_pts_to_ref = ref.PointsByIndices(nns.indices);

    results.trans = Utils::ComputeBestFitTransform(current_src, closest_src_pts_to_ref);
    current_src = current_src.Transformed(results.trans);

    results.error = std::accumulate(nns.distances.begin(), nns.distances.end(), 0.0) / nns.distances.size();
    diff = fabs(results.error - prev_error);
    if (diff < tolerance) {
      break;
    }
    prev_error = results.error;
  }
  
  results.converged = fabs(results.error - prev_error) < tolerance;
  results.trans = Utils::ComputeBestFitTransform(src, current_src);

  return results;
}

} // namespace Utils
} // namespace ICP
