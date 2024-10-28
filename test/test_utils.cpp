#include <gtest/gtest.h>
#include "icp/icp_cloud.hpp"
#include "icp/utils.hpp"
#include "testing/test_helpers.hpp"

TEST(ICPUtilsTest, ICPSolver) {

    double width=3.0;
    double height=4.6;

    int n_pts_w_src = 33;
    int n_pts_h_src = 54;

    int n_pts_w_ref = 43;
    int n_pts_h_ref = 65;


    ICP::ICPPointCloud ref_plane = ICP::GenerateRectangularPlane(
        ICP::GenerateTransformationMatrix(0., 0., 0., 0., -M_PI_4, 0.), 
        width,
        height,
        n_pts_w_src,
        n_pts_h_src);

    ICP::ICPPointCloud src_plane = ICP::GenerateRectangularPlane(
        ICP::GenerateTransformationMatrix(1., 1., 1., 0., 0., 0.), 
        width,
        height,
        n_pts_w_ref,
        n_pts_h_ref);

    ICP::Mat4 expected_transform = ICP::GenerateTransformationMatrix(1., 1., 1., 0.,M_PI_4,0.);
    ICP::ICPResults results = ICP::Utils::MatchSourceScanToReferenceScan(src_plane, ref_plane, 50, 0.0001);

    cout << expected_transform << endl;
    cout <<endl<<endl<<results<<endl;

    EXPECT_TRUE(false);

}