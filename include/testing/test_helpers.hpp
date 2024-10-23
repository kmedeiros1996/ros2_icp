#pragma once

#include "icp/types.hpp"
#include <assert.h>

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

Mat4 GenerateTransformationMatrix(double x, double y, double z, double qx, double qy, double qz, double qw) {
    Mat4 tmatrix = Eigen::Matrix4d::Identity();
    tmatrix.block<3,3>(0,0) = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    tmatrix.block<3,1>(0,3) = Eigen::Vector3d(x, y, z);
    return tmatrix;
}

ICPPointCloud GenerateCircle(const Mat4 tmatrix, const double radius, const int num_pts) {
    assert(num_pts > 1);

    PointCloudMat base = Eigen::MatrixXd::Ones(num_pts, 3);
    double angle = 0;
    double angle_incr = M_2_PI / static_cast<double>(num_pts);
    for (int i = 0; i < num_pts; i++) {
        base.row(i) = Vec3(radius * cos(angle), radius * sin(angle), 0.0);
        angle += angle_incr;

    }

    ICPPointCloud out = ICPPointCloud(base).Transformed(tmatrix);

    return out;
}

ICPPointCloud GenerateRectangularPlane(const Mat4 tmatrix, const double width, const double height, const int num_pts_w, const int num_pts_h) {
    double w_incr = width / static_cast<double> (num_pts_w);
    double h_incr = height / static_cast<double> (num_pts_h);
    PointCloudMat base = Eigen::MatrixXd::Ones(num_pts_w * num_pts_h, 3);

    for (int ih = 0; ih < num_pts_h; ih++) {
        for (int iw = 0; iw < num_pts_w; iw++) {
            int idx1d = num_pts_w * ih + iw;
            base.row(idx1d) = Vec3(iw * w_incr, ih * h_incr, 0.0);
        }
    }

    ICPPointCloud out = ICPPointCloud(base).Transformed(tmatrix);
    return out;
}

} // namespace ICP 
