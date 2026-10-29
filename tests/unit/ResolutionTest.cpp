#include <ConstraintGeometry/constraint/BilateralResolution.h>
#include <ConstraintGeometry/constraint/InsertionResolution.h>
#include <ConstraintGeometry/constraint/UnilateralResolution.h>
#include <gtest/gtest.h>

#include <cmath>

using namespace sofa::constraintgeometry;

namespace {

void expectFinite(const double* values, unsigned count) {
    for (unsigned i = 0; i < count; i++) {
        EXPECT_TRUE(std::isfinite(values[i])) << "value " << i << " is not finite";
    }
}

}  // namespace

TEST(ResolutionTest, BilateralResolutionIgnoresZeroDiagonal) {
    BilateralConstraintResolution1 resolution;

    double matrix[1][1] = {{0.0}};
    double* rows[1] = {matrix[0]};
    double delta[1] = {1.0};
    double force[1] = {0.0};

    resolution.resolution(0, rows, delta, force, nullptr);

    expectFinite(force, 1);
    EXPECT_DOUBLE_EQ(force[0], 0.0);
}

TEST(ResolutionTest, BilateralSystemFallsBackOnSingularMatrix) {
    BilateralConstraintResolution2 resolution;

    double matrix[2][2] = {{1.0, 1.0}, {1.0, 1.0}};
    double* rows[2] = {matrix[0], matrix[1]};
    double delta[2] = {1.0, 2.0};
    double force[2] = {0.0, 0.0};

    resolution.init(0, rows, force);
    resolution.resolution(0, rows, delta, force, nullptr);

    expectFinite(force, 2);
}

TEST(ResolutionTest, UnilateralResolutionIgnoresZeroDiagonal) {
    UnilateralConstraintResolution resolution;

    double matrix[1][1] = {{0.0}};
    double* rows[1] = {matrix[0]};
    double delta[1] = {1.0};
    double force[1] = {0.0};

    resolution.resolution(0, rows, delta, force, nullptr);

    expectFinite(force, 1);
    EXPECT_DOUBLE_EQ(force[0], 0.0);
}

TEST(ResolutionTest, UnilateralFrictionResolutionIgnoresZeroDiagonal) {
    UnilateralFrictionResolution resolution(0.5);

    double matrix[3][3] = {{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    double* rows[3] = {matrix[0], matrix[1], matrix[2]};
    double delta[3] = {1.0, 2.0, 3.0};
    double force[3] = {0.0, 0.0, 0.0};

    resolution.resolution(0, rows, delta, force, nullptr);

    expectFinite(force, 3);
}

TEST(ResolutionTest, UnilateralFrictionClampsTangentialForcesSymmetrically) {
    UnilateralFrictionResolution resolution(0.5);

    double matrix[3][3] = {{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
    double* rows[3] = {matrix[0], matrix[1], matrix[2]};
    double delta[3] = {-2.0, -100.0, 100.0};
    double force[3] = {0.0, 0.0, 0.0};

    resolution.resolution(0, rows, delta, force, nullptr);

    expectFinite(force, 3);
    EXPECT_DOUBLE_EQ(force[0], 1.0);
    EXPECT_DOUBLE_EQ(force[1], 0.5);
    EXPECT_DOUBLE_EQ(force[2], -0.5);
}

TEST(ResolutionTest, UnilateralFrictionConeBoundsTheNormOfTheTangentialForce) {
    // same data as the pyramid test: the pyramid keeps (0.5, -0.5), norm 0.707 > mu * fn = 0.5; the cone scales it to 0.5
    UnilateralFrictionResolution resolution(0.5, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                                            std::numeric_limits<double>::max(), true);

    double matrix[3][3] = {{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
    double* rows[3] = {matrix[0], matrix[1], matrix[2]};
    double delta[3] = {-2.0, -100.0, 100.0};
    double force[3] = {0.0, 0.0, 0.0};

    resolution.resolution(0, rows, delta, force, nullptr);

    expectFinite(force, 3);
    EXPECT_DOUBLE_EQ(force[0], 1.0);
    EXPECT_NEAR(std::hypot(force[1], force[2]), 0.5, 1e-12);
    EXPECT_NEAR(force[1], 0.5 / std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(force[2], -0.5 / std::sqrt(2.0), 1e-12);
}

TEST(ResolutionTest, UnilateralFrictionConeLeavesStickingForcesUntouched) {
    UnilateralFrictionResolution cone(0.5, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max(), true);
    UnilateralFrictionResolution pyramid(0.5);

    for (UnilateralFrictionResolution* resolution : {&cone, &pyramid}) {
        double matrix[3][3] = {{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
        double* rows[3] = {matrix[0], matrix[1], matrix[2]};
        double delta[3] = {-2.0, -0.4, 0.2};  // tangential force (0.2, -0.1): norm 0.22 < 0.5
        double force[3] = {0.0, 0.0, 0.0};
        resolution->resolution(0, rows, delta, force, nullptr);
        EXPECT_DOUBLE_EQ(force[0], 1.0);
        EXPECT_NEAR(force[1], 0.2, 1e-12);
        EXPECT_NEAR(force[2], -0.1, 1e-12);
    }
}

TEST(ResolutionTest, InsertionResolutionHandlesSingularMatrix) {
    InsertionConstraintResolution resolution(0.5);

    double matrix[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}};
    double* rows[3] = {matrix[0], matrix[1], matrix[2]};
    double delta[3] = {1.0, 2.0, 3.0};
    double force[3] = {0.0, 0.0, 0.0};

    resolution.init(0, rows, force);
    resolution.resolution(0, rows, delta, force, nullptr);

    expectFinite(force, 3);
}
