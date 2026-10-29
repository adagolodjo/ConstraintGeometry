#include <ConstraintGeometry/ConstraintNormal.h>
#include <gtest/gtest.h>

// Basic test to ensure the test framework is working.
TEST(ConstraintGeometryTest, SanityCheck) {
    const sofa::constraintgeometry::ConstraintNormal normal(sofa::type::Vec3(1, 0, 0));

    EXPECT_EQ(normal.size(), 1u);
}
