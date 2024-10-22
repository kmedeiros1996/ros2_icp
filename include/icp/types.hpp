/*
types.hpp
Author: Skye Medeiros

In C++, I like to add syntactic sugar where it makes sense, 
so type shortcuts like these make me happy

*/
#pragma once

#include "nanoflann/nanoflann.hpp"
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <vector>
#include <unordered_map>
#include <iostream>

using std::cout; 
using std::endl;


namespace ICP {
using PointCloudMat = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using HomogeneousPointCloudMat = Eigen::Matrix<double, Eigen::Dynamic, 4>;

// This software will stick with the convention of points being stored as (3 * 1)
// and pointclouds stored as (3 * n_pts).
// Not only will this be nice sugar for these types, 
// but also help make some nice compile time guarantees
// by using the fixed size Eigen matrix classes.
using PointCloudMatTranspose = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using HomogeneousPointCloudMatTranspose = Eigen::Matrix<double, 4, Eigen::Dynamic>;
using Vec3 = Eigen::RowVector3d;
using Vec4 = Eigen::RowVector4d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;

using NanoFlannKNNResults = nanoflann::KNNResultSet<double>;
using NearestNeighborKDTree = nanoflann::KDTreeEigenMatrixAdaptor<PointCloudMat>;

struct NearestNeighborsResult {
    int index;
    double distance;
};

using SizeTVector = std::vector<size_t>;
using IntVector = std::vector<double>;
using DoubleVector = std::vector<double>;
using Vec3Vector = std::vector<Vec3>;
using NearestNeighborsResultVector = std::vector<NearestNeighborsResult>;


using ROSLaserScan = sensor_msgs::msg::LaserScan;
using ROSPointCloud = sensor_msgs::msg::PointCloud2;
using ROSTF = geometry_msgs::msg::TransformStamped;
using ROSOdometry = nav_msgs::msg::Odometry;

} // namespace ICP
