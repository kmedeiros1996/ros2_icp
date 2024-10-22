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
        shifted_m.row(i) = points_.row(i) - point;
    }

    return ICPPointCloud(shifted_m);
}

ICPPointCloud ICPPointCloud::Centered(){
    return Shifted(Centroid());
}

HomogeneousPointCloudMat ICPPointCloud::TransformedHomogeneous(const Mat4& transf){
    return (transf * Homogeneous().transpose()).transpose();
}

ICPPointCloud ICPPointCloud::Transformed(const Mat4& transf) {
    return ICPPointCloud(TransformedHomogeneous(transf));
}

std::unique_ptr<NearestNeighborKDTree> ICPPointCloud::BuildKDTree() const {
    return std::make_unique<NearestNeighborKDTree> (3, std::cref(points_), 10);
}

NearestNeighborsResult ICPPointCloud::GetNearestNeighborsResult(const Vec3& q){
    if (!HasKDTree()) {
        kd_tree_ = BuildKDTree();
    }
    SizeTVector temp_index(1);
    DoubleVector temp_dist(1);

    NanoFlannKNNResults knn_results(1);
    knn_results.init(&temp_index[0], &temp_dist[0]);
    DoubleVector query_pt{q(0), q(1), q(2)};
    kd_tree_->index->findNeighbors(knn_results, &query_pt[0], nanoflann::SearchParams(10));
    
    NearestNeighborsResult nn;
    nn.distance = temp_dist[0];
    nn.index = int(temp_index[0]);

    return nn;
}

NearestNeighborsResultVector ICPPointCloud::GetNearestNeighborsResults(const Vec3Vector& qs) {
    NearestNeighborsResultVector results;
    for (const Vec3& q : qs) {
        results.push_back(GetNearestNeighborsResult(q));
    }

    return results;
}

Vec3 ICPPointCloud::GetNearestPoint(const Vec3& q) {
    NearestNeighborsResult n = GetNearestNeighborsResult(q);
    return PointAt(n.index);
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

bool ICPPointCloud::HasKDTree() const {
    return kd_tree_ != nullptr;
}

} // namespace ICP