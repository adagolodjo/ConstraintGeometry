#include <ConstraintGeometry/ConstraintNormal.h>
#include <gtest/gtest.h>

#include <cmath>

using namespace sofa::constraintgeometry;
using sofa::type::Vec3;

namespace {

void expectOrthonormal(const ConstraintNormal& normal) {
    ASSERT_EQ(normal.size(), 3u);

    for (unsigned i = 0; i < normal.size(); i++) {
        EXPECT_NEAR(normal[i].norm(), 1.0, 1e-12);
    }

    for (unsigned i = 0; i < normal.size(); i++) {
        for (unsigned j = i + 1; j < normal.size(); j++) {
            EXPECT_NEAR(dot(normal[i], normal[j]), 0.0, 1e-12);
        }
    }
}

}  // namespace

TEST(ConstraintNormalTest, ZeroDirectionIsIgnored) {
    ConstraintNormal normal(Vec3(0, 0, 0));
    EXPECT_EQ(normal.size(), 0u);

    normal.add(Vec3(0, 0, 0));
    EXPECT_EQ(normal.size(), 0u);
}

TEST(ConstraintNormalTest, DirectionIsNormalized) {
    ConstraintNormal normal(Vec3(0, 0, 2));
    ASSERT_EQ(normal.size(), 1u);
    EXPECT_NEAR(normal[0].norm(), 1.0, 1e-12);
    EXPECT_NEAR(normal[0][2], 1.0, 1e-12);
}

TEST(ConstraintNormalTest, OrthogonalDirectionsAreOrthonormal) {
    ConstraintNormal normal(Vec3(1, 0, 0));
    normal.addOrthogonalDirection();
    normal.addOrthogonalDirection();

    expectOrthonormal(normal);
}

TEST(ConstraintNormalTest, OrthogonalDirectionsFromEmptyDirection) {
    ConstraintNormal normal;
    normal.addOrthogonalDirection();
    normal.addOrthogonalDirection();
    normal.addOrthogonalDirection();

    expectOrthonormal(normal);
}

TEST(ConstraintNormalTest, CopyAndAssignmentPreserveDirections) {
    ConstraintNormal normal(Vec3(1, 2, 3));

    ConstraintNormal copy(normal);
    ASSERT_EQ(copy.size(), 1u);
    EXPECT_NEAR(copy[0].norm(), 1.0, 1e-12);

    ConstraintNormal assigned;
    assigned = normal;
    ASSERT_EQ(assigned.size(), 1u);
    EXPECT_NEAR(assigned[0].norm(), 1.0, 1e-12);
}
