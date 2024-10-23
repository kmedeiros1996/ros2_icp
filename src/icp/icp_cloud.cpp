/*
icp_cloud.cpp
Author: Skye Medeiros

Class implementations for "icp_cloud.hpp"
*/
#include <icp/icp_cloud.hpp>
#include <assert.h>
namespace ICP
{
ICPPointCloud::ICPPointCloud(const PointCloudMat& cloud) : points_{cloud}
{}

ICPPointCloud::ICPPointCloud(const ICPPointCloud& other) : points_{other.Points()} {
    if (other.HasCentroid()) {
        centroid_.emplace(other.centroid_.value());
    }

    if (other.HasHomogeneous()) {
        homo_.emplace(other.homo_.value());
    }

}

ICPPointCloud& ICPPointCloud::operator=(const ICPPointCloud& other) {
    this->points_ = other.points_;
    this->centroid_ = other.centroid_;
    this->homo_ = other.homo_;

    return *this;
}

ICPPointCloud::ICPPointCloud(const HomogeneousPointCloudMat& homo_cloud)
{
    points_ = PointCloudMat::Ones(homo_cloud.rows(), 3);
    for (int i = 0; i < points_.rows(); i++) {
        
        double x = homo_cloud(i, 0);
        double y = homo_cloud(i, 1);
        double z = homo_cloud(i, 2);
        double w = homo_cloud(i, 3);
        
        assert(w != 0);

        points_.row(i) = Vec3(x/w, y/w, z/w);
    }
    homo_.emplace(homo_cloud);
}

PointCloudMat ICPPointCloud::Points() const {
    return points_;
}

Vec3 ICPPointCloud::PointAt(int idx) const {
    return points_.row(idx).transpose();
}

Vec4 ICPPointCloud::HomogeneousPointAt(int idx) {
    return Homogeneous().row(idx);
}

Vec3 ICPPointCloud::Centroid() {
    if (!HasCentroid()) {
        Vec3 centroid (0.,0.,0.);

        for (int i = 0; i < points_.rows(); i++){
            centroid += PointAt(i);
        }
        centroid = centroid / points_.rows();
        centroid_.emplace(centroid);
    }
    return centroid_.value();
}

HomogeneousPointCloudMat ICPPointCloud::Homogeneous() {
    if (!HasHomogeneous()) {
        HomogeneousPointCloudMat homo = Eigen::MatrixXd::Ones(points_.rows(), 4);
        homo.leftCols(3) = points_;
        homo_.emplace(homo);
    }

    return homo_.value();
}

PointCloudMatTranspose ICPPointCloud::Transposed() {
    if (!HasTranspose()) {
        trans_.emplace(points_.transpose());
    }
    return trans_.value();
}

ICPPointCloud ICPPointCloud::Shifted(const Vec3& point) {
    PointCloudMat shifted_m = points_;
    for (int i = 0; i < points_.rows(); i++) {
        shifted_m.row(i) = PointAt(i) - point;
    }

    return ICPPointCloud(shifted_m);
}

ICPPointCloud ICPPointCloud::Centered(){
    return Shifted(Centroid());
}

ICPPointCloud ICPPointCloud::PointsByIndices(const IntVector& idxes) {
    PointCloudMat ordered = Eigen::MatrixXd::Ones(idxes.size(), 3);

    for (int i = 0; i < idxes.size(); i++) {
        assert (idxes[i] < points_.rows());
        ordered.row(i) = PointAt(idxes[i]);
    }

    return ICPPointCloud(ordered);
}

HomogeneousPointCloudMat ICPPointCloud::TransformedHomogeneous(const Mat4& transf){
    return (transf * Homogeneous().transpose()).transpose();
}

ICPPointCloud ICPPointCloud::Transformed(const Mat4& transf) {
    return ICPPointCloud(TransformedHomogeneous(transf));
}

ICP::KDTree ICPPointCloud::ToKDTree() const {
    return ICP::KDTree(points_);
}

bool ICPPointCloud::HasHomogeneous() const {
    return homo_.has_value();
}

bool ICPPointCloud::HasCentroid() const {
    return centroid_.has_value();
}

bool ICPPointCloud::HasTranspose() const {
    return trans_.has_value();
}

} // namespace ICP