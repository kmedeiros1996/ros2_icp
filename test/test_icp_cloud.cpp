
#include <gtest/gtest.h>
#include "icp/icp_cloud.hpp"
#include <testing/test_helpers.hpp>

#include <iostream>

TEST(ICPCloudTest, Construction) {
    ICP::PointCloudMat pc_m {{1., 2., 3.}, {4., 5., 6.}, {7., 8., 9.}};
    ICP::ICPPointCloud pc(pc_m);

    EXPECT_FALSE(pc.HasHomogeneous());
    EXPECT_FALSE(pc.HasCentroid());

    ICP::HomogeneousPointCloudMat homo_pc_m {{1., 2., 3., 1.}, {4., 5., 6., 1.}, {7., 8., 9., 1.}};
    ICP::ICPPointCloud homo_pc(homo_pc_m);

    EXPECT_TRUE(homo_pc.HasHomogeneous());
    EXPECT_FALSE(homo_pc.HasCentroid());

    for (int i = 0; i < pc_m.rows(); i++)
    {
        EXPECT_EQ(pc.PointAt(i), pc_m.row(i));
        EXPECT_EQ(pc.Homogeneous(), homo_pc_m);
        EXPECT_EQ(homo_pc.PointAt(i), pc_m.row(i));
    }
}

TEST(ICPCloudTest, Centroid) {
    std::vector<ICP::PointCloudMat> pc_mats {
        ICP::PointCloudMat{{1., 1., 1.}, {2., 2., 2.}, {3., 3., 3.}},
        ICP::PointCloudMat{{1.5, 1.996, 1.9}, {7.3, 2.3, 8.}, {3.1, 3.17, 12.1}}
    };

    std::vector<ICP::Vec3> expected_centroids {
        ICP::Vec3(2., 2., 2.),
        ICP::Vec3(3.9667, 2.4889, 7.3333)
    };

    for (int i = 0; i < pc_mats.size(); i++) {
        ICP::ICPPointCloud pc(pc_mats[i]);
        EXPECT_TRUE(ICP::AreVec3sClose(pc.Centroid(), expected_centroids[i], 0.001));
    }
}

TEST(ICPCloudTest, Shifting) {
    ICP::PointCloudMat pc_m {{1., 2., 3.}, {4., 5., 6.}, {7., 8., 9.}};
    ICP::PointCloudMat expected_shifted_pc {{0., 1., 2.}, {3., 4., 5.}, {6., 7., 8.}};
    ICP::ICPPointCloud pc(pc_m);
    ICP::Vec3 shifter{1.0, 1.0, 1.0};

    ICP::PointCloudMat pc_m_shifted = pc.Shifted(shifter).Points();

    for (int i = 0; i < pc_m.rows(); i++) {
        EXPECT_TRUE(ICP::AreVec3sClose(expected_shifted_pc.row(i), pc_m_shifted.row(i), 0.001));
    }
}

TEST(ICPCloudTest, Centering) {
    ICP::ICPPointCloud pc (ICP::PointCloudMat{{1., 1., 1.}, {2., 2., 2.}, {3., 3., 3.}});

    ICP::PointCloudMat expected_centered_cloud{{-1., -1., -1.}, {0., 0., 0.}, {1., 1., 1.}};
    ICP::PointCloudMat centered_pts = pc.Centered().Points();

    for (int i = 0; i < centered_pts.rows(); i++) {
        EXPECT_TRUE(ICP::AreVec3sClose(centered_pts.row(i), expected_centered_cloud.row(i)));
    }
}

TEST(ICPCloudTest, Transformation) {
    ICP::Mat4 tmatr = ICP::GenerateTransformationMatrix(0.0, 0.0, -1.0, 0.0, 0.0, M_PI_4);
    ICP::PointCloudMat pc_m {{1., 0., 0.}, 
                             {0., 1., 0.}, 
                             {0., -1., 0.}, 
                             {-1., 0., 0.}};
    ICP::PointCloudMat expected_pc_trans_m {{0.707107, 0.707107, -1}, 
                                            {-0.707107,0.707107,-1}, 
                                            {0.707107,-0.707107,-1}, 
                                            {-0.707107,-0.707107,-1}};

    ICP::PointCloudMat pc_trans = ICP::ICPPointCloud(pc_m).Transformed(tmatr).Points();

    for (int i = 0; i < pc_m.rows(); i++) {
        EXPECT_TRUE(ICP::AreVec3sClose(pc_trans.row(i), expected_pc_trans_m.row(i)));
    }
}

TEST(ICPCloudTest, PointsByIndices) {
    ICP::ICPPointCloud pc (ICP::PointCloudMat{{1., 1., 1.}, 
                                              {2., 2., 2.}, 
                                              {4., 4., 4.}, 
                                              {3., 3., 3.}});

    ICP::IntVector idxes{1, 2, 1, 2, 0, 0};
    ICP::ICPPointCloud ordered = pc.PointsByIndices(idxes);

    for (int i = 0; i < idxes.size(); i++) {
        EXPECT_EQ(ordered.PointAt(i), pc.PointAt(idxes[i]));
    }
}

TEST(ICPCloudTest, KDTree) 
{
    ICP::ICPPointCloud pc (ICP::PointCloudMat{{1., 1., 1.}, {2., 2., 2.}, {3., 3., 3.}});

    ICP::Vec3Vector query_pts{{1.25, 1.25, 1.25},
                              {2.96, 3.14, 1.99},
                              {1.73, 0.99, 2.001}};
    ICP::IntVector expected_indices{0, 2, 1};

    ICP::KDTree pc_tree = pc.ToKDTree();
    cout <<"made tree"<<endl;
    ICP::NearestNeighborsResults results = pc_tree.GetNearestNeighborsResults(query_pts);
    cout<<"got results"<<endl;
    for (int i = 0; i < results.indices.size(); i++) {
        EXPECT_EQ(results.indices[i], expected_indices[i]);
    }

}