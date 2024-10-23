#include "icp/kd_tree.hpp"

namespace ICP {
KDTree::KDTree(const PointCloudMat& points) {
    kd_tree_ = std::make_unique<NearestNeighborKDTree> (3, std::cref(points), 10);
}

NearestNeighborsResult KDTree::GetNearestNeighborsResult(const Vec3& q){

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

NearestNeighborsResults KDTree::GetNearestNeighborsResults(const Vec3Vector& qs) {
    NearestNeighborsResults results;
    for (const Vec3& q : qs) {
        const auto result = GetNearestNeighborsResult(q);
        results.indices.push_back(result.index);
        results.distances.push_back(result.distance);
    }

    return results;
}

NearestNeighborsResults KDTree::GetNearestNeighborsResults(const PointCloudMat& m) {
    NearestNeighborsResults results;
    for (int i = 0; i < m.rows(); i++) {
        const auto result = GetNearestNeighborsResult(m.row(i));
        results.indices.push_back(result.index);
        results.distances.push_back(result.distance);
    }

    return results;
}

} // namespace ICP