/*
icp_cloud.hpp
Author: Skye Medeiros

Convenience class which shortcuts some often used calculations and data structures.

*/

#include <icp/types.hpp>

#include <optional>
#include <memory>

namespace ICP {

class ICPPointCloud
{
    public:

    // Default constructor with a (num_pts * 3) matrix
    ICPPointCloud(const PointCloudMat& cloud);

    // For convenient transformations,
    // ICPPointClouds can also be constructed from a homogeneous pointcloud (num_pts * 4)
    // which initializes both the euclidean AND homogeneous point set.
    ICPPointCloud(const HomogeneousPointCloudMat& homo_cloud); 

    PointCloudMat                            Points() const;
    Vec3                                     PointAt(int idx) const;

    // The homogeneous cloud is lazily initialized and only computed once.
    // Returns the homogeneous point at an index.
    Vec4                                     HomogeneousPointAt(int idx);

    // The centroid is lazily initialized and only computed once.
    // Returns the average of all contained points.
    Vec3                                     Centroid();

    // Builds and caches the pointcloud in homogeneous coordinates if it doesn't already exist.
    // Returns the point data in heterogeneous coordinates (num_pts * 4)
    HomogeneousPointCloudMat                 Homogeneous();

    // Builds and caches the transposed pointcloud if it doesn't already exist.
    // Returns the matrix containing the underlying pointcloud data, transposed.
    PointCloudMatTranspose                   Transposed();

    // Returns a new ICPPointCloud with the points shifted by a given point.
    ICPPointCloud                            Shifted(const Vec3& point);

    // Builds and caches the pointcloud in homogeneous coordinates if it doesn't already exist.
    // Returns a HomogeneousPointCloudMat transformed by a given 4D transformation matrix.
    HomogeneousPointCloudMat                 TransformedHomogeneous(const Mat4& transf);
    ICPPointCloud                            Transformed(const Mat4& transf);

    // Builds and caches the centroid AND pointcloud in homogeneous coordinates if they don't exist.
    ICPPointCloud                            Centered();
    
    // Builds and caches the KD Tree if it doesn't exist.
    // Returns a struct containing the index of and distance to the closest point.
    NearestNeighborsResult                   GetNearestNeighborsResult(const Vec3& q);
    NearestNeighborsResultVector             GetNearestNeighborsResults(const Vec3Vector& qs);

    // Builds and caches the KD Tree if it doesn't exist.
    // Returns the closest point in this pointcloud to a query point.
    Vec3                                     GetNearestPoint(const Vec3& q);

    bool                                     HasHomogeneous() const;
    bool                                     HasCentroid() const;
    bool                                     HasKDTree() const;
    bool                                     HasTranspose() const;

    private:
    std::unique_ptr<NearestNeighborKDTree>   BuildKDTree() const;

    private:

    // Every ICPPointCloud will possess a PointCloudMat at minimum.
    // The KD Tree, centroid, and pointcloud in heterogenous coordinates
    // are all lazily initialized.

    PointCloudMat                            points_; 
    std::optional<Vec3>                      centroid_;
    std::optional<PointCloudMatTranspose>    trans_;
    std::optional<HomogeneousPointCloudMat>  homo_;
    std::unique_ptr<NearestNeighborKDTree>   kd_tree_;
};
} // namespace ICP