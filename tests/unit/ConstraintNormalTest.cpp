#include <CollisionAlgorithm/proximity/FixedProximity.h>
#include <ConstraintGeometry/ConstraintNormal.h>
#include <gtest/gtest.h>
#include <sofa/linearalgebra/FullVector.h>

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

TEST(ConstraintNormalTest, ViolationOffsetShiftsTheGapAlongTheDirection) {
    using sofa::collisionalgorithm::FixedProximity;

    // P is 5 above Q along +Z: with a contact thickness of 4.5 the gap is still open by 0.5
    auto first = std::make_shared<FixedProximity>(Vec3(0, 0, 5));
    auto second = std::make_shared<FixedProximity>(Vec3(0, 0, 0));

    ConstraintNormal normal(Vec3(0, 0, 1));
    sofa::linearalgebra::FullVector<double> v(2);

    normal.computeViolations(0, first, second, &v);
    EXPECT_NEAR(v[0], 5.0, 1e-12);

    normal.addViolationOffset(0, 4.5);
    normal.computeViolations(0, first, second, &v);
    EXPECT_NEAR(v[0], 0.5, 1e-12);

    // thicker than the gap: the constraint is violated (negative gap)
    normal.addViolationOffset(0, 1.0);
    normal.computeViolations(0, first, second, &v);
    EXPECT_NEAR(v[0], -0.5, 1e-12);

    // out-of-range index and null offset are ignored
    normal.addViolationOffset(3, 10.0);
    normal.addViolationOffset(0, 0.0);
    normal.computeViolations(0, first, second, &v);
    EXPECT_NEAR(v[0], -0.5, 1e-12);
}

TEST(ConstraintNormalTest, SetViolationFuncReplacesTheFunction) {
    using sofa::collisionalgorithm::FixedProximity;

    auto first = std::make_shared<FixedProximity>(Vec3(0, 0, 5));
    auto second = std::make_shared<FixedProximity>(Vec3(0, 0, 0));

    ConstraintNormal normal(Vec3(0, 0, 1));
    normal.setViolationFunc(0, [](const ConstraintNormal::FIRST::SPtr&, const ConstraintNormal::SECOND::SPtr&,
                                  const Vec3&) { return 42.0; });
    sofa::linearalgebra::FullVector<double> v(1);
    normal.computeViolations(0, first, second, &v);
    EXPECT_NEAR(v[0], 42.0, 1e-12);
}

TEST(ConstraintNormalTest, TransverseDirectionsAreOrthogonalToTheAxis) {
    for (const Vec3 axis : {Vec3(1, 0, 0), Vec3(0, 0, 3), Vec3(0, 2, 0), Vec3(1, 2, -3)}) {
        const ConstraintNormal normal = ConstraintNormal::transverse(axis);
        ASSERT_EQ(normal.size(), 2u);
        EXPECT_NEAR(normal[0].norm(), 1.0, 1e-12);
        EXPECT_NEAR(normal[1].norm(), 1.0, 1e-12);
        EXPECT_NEAR(dot(normal[0], normal[1]), 0.0, 1e-12);
        EXPECT_NEAR(dot(normal[0], axis.normalized()), 0.0, 1e-12);
        EXPECT_NEAR(dot(normal[1], axis.normalized()), 0.0, 1e-12);
    }
    EXPECT_EQ(ConstraintNormal::transverse(Vec3(0, 0, 0)).size(), 0u);
}
