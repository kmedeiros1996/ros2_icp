#include "icp/types.hpp"

namespace ICP {
bool AreVec3sClose(const Vec3& v1, const Vec3& v2, double epsilon=1e-5) {
    return (v2 - v1).norm() < epsilon;
}

Mat3 GenerateRotationMatrix(double roll_radians, double pitch_radians, double yaw_radians) {
    return (Eigen::AngleAxisd(roll_radians, Eigen::Vector3d::UnitX()) 
           * Eigen::AngleAxisd(pitch_radians, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(yaw_radians, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

Mat4 GenerateTransformationMatrix(double x, double y, double z, double roll_radians, double pitch_radians, double yaw_radians) {
    Mat4 tmatrix = Eigen::Matrix4d::Identity();
    tmatrix.block<3,1>(0,3) = Eigen::Vector3d(x, y, z);
    tmatrix.block<3,3>(0,0) = GenerateRotationMatrix(roll_radians, pitch_radians, yaw_radians);
    return tmatrix;
}
} // namespace ICP 
