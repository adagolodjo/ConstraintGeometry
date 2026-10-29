#include <ConstraintGeometry/ConstraintProximity.h>
#include <ConstraintGeometry/InternalConstraint.h>
#include <gtest/gtest.h>

#include <memory>

using namespace sofa::constraintgeometry;
using sofa::type::Vec3;

namespace {

class MockConstraintProximity : public ConstraintProximity {
public:
    typedef std::shared_ptr<MockConstraintProximity> SPtr;

    MockConstraintProximity(const Vec3& position, const Vec3& normal) : m_position(position), m_normal(normal) {
    }

    Vec3 getPosition(sofa::core::VecCoordId = sofa::core::vec_id::write_access::position) const override {
        return m_position;
    }

    Vec3 getVelocity(sofa::core::VecDerivId = sofa::core::vec_id::write_access::velocity) const override {
        return Vec3();
    }

    Vec3 getNormal() const override {
        return m_normal;
    }

private:
    Vec3 m_position;
    Vec3 m_normal;
};

ConstraintNormal createNormal(const MockConstraintProximity::SPtr&, const MockConstraintProximity::SPtr&) {
    return ConstraintNormal(Vec3(1, 0, 0));
}

sofa::core::behavior::ConstraintResolution* createResolution(const BaseInternalConstraint*) {
    return nullptr;
}

}  // namespace

TEST(InternalConstraintTest, ExposesPairsAndSupportsSelfAssignment) {
    auto first = std::make_shared<MockConstraintProximity>(Vec3(0, 0, 0), Vec3(1, 0, 0));
    auto second = std::make_shared<MockConstraintProximity>(Vec3(1, 0, 0), Vec3(1, 0, 0));

    auto constraint = InternalConstraint<MockConstraintProximity, MockConstraintProximity>::create(
        first, second, createNormal, createResolution);

    ASSERT_EQ(constraint->getPairSize(), 1u);
    ASSERT_EQ(constraint->size(), 1u);
    EXPECT_NE(constraint->getFirstPair(0), nullptr);
    EXPECT_NE(constraint->getSecondPair(0), nullptr);

    // Self-assignment must preserve the pairs instead of clearing them first.
    constraint->operator=(*constraint);

    EXPECT_EQ(constraint->getPairSize(), 1u);
    EXPECT_EQ(constraint->size(), 1u);
    EXPECT_NE(constraint->getFirstPair(0), nullptr);
    EXPECT_NE(constraint->getSecondPair(0), nullptr);
}

TEST(InternalConstraintTest, OutOfRangePairReturnsNull) {
    auto first = std::make_shared<MockConstraintProximity>(Vec3(0, 0, 0), Vec3(1, 0, 0));
    auto second = std::make_shared<MockConstraintProximity>(Vec3(1, 0, 0), Vec3(1, 0, 0));

    auto constraint = InternalConstraint<MockConstraintProximity, MockConstraintProximity>::create(
        first, second, createNormal, createResolution);

    EXPECT_EQ(constraint->getFirstPair(10), nullptr);
    EXPECT_EQ(constraint->getSecondPair(10), nullptr);
}
