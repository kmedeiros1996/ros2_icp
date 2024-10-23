#pragma once

#include "icp/types.hpp"

namespace ICP {

struct NearestNeighborsResult {
    int index;
    double distance;
};

struct NearestNeighborsResults {
    IntVector indices;
    DoubleVector distances;
};

class KDTree {
public:

    KDTree(const PointCloudMat& points);
    // Returns a struct containing the index of and distance to the closest point.
    NearestNeighborsResult                   GetNearestNeighborsResult(const Vec3& q);
    NearestNeighborsResults                  GetNearestNeighborsResults(const Vec3Vector& qs);
    NearestNeighborsResults                  GetNearestNeighborsResults(const PointCloudMat& m);
    
private:
    std::unique_ptr<NearestNeighborKDTree>   kd_tree_;
};
} // namespace ICP