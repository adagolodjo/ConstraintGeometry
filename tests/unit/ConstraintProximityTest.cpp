#include <ConstraintGeometry/ConstraintProximity.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

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

void expectVec3(const Vec3& actual, const Vec3& expected) {
    EXPECT_NEAR(actual[0], expected[0], 1e-12);
    EXPECT_NEAR(actual[1], expected[1], 1e-12);
    EXPECT_NEAR(actual[2], expected[2], 1e-12);
}

}  // namespace

TEST(ConstraintProximityTest, EmptyMultiProximityIsNeutral) {
    ConstraintProximity::SPtr multi = MultiConstraintProximity::create({});

    expectVec3(multi->getPosition(), Vec3());
    expectVec3(multi->getVelocity(), Vec3());
    expectVec3(multi->getNormal(), Vec3());

    // Must not dereference the empty container.
    multi->buildJacobianConstraint(sofa::core::MultiMatrixDerivId(), {}, 1.0, 0);
}

TEST(ConstraintProximityTest, MultiProximityAveragesPositionAndNormal) {
    std::vector<ConstraintProximity::SPtr> proximities = {
        std::make_shared<MockConstraintProximity>(Vec3(0, 0, 0), Vec3(1, 0, 0)),
        std::make_shared<MockConstraintProximity>(Vec3(2, 0, 0), Vec3(0, 1, 0)),
    };

    ConstraintProximity::SPtr multi = MultiConstraintProximity::create(proximities);

    expectVec3(multi->getPosition(), Vec3(1, 0, 0));
    expectVec3(multi->getNormal(), Vec3(0.5, 0.5, 0));
}

TEST(ConstraintProximityTest, MultiProximityIgnoresNullEntries) {
    std::vector<ConstraintProximity::SPtr> proximities = {
        nullptr,
        std::make_shared<MockConstraintProximity>(Vec3(2, 0, 0), Vec3(0, 1, 0)),
    };

    ConstraintProximity::SPtr multi = MultiConstraintProximity::create(proximities);

    expectVec3(multi->getPosition(), Vec3(2, 0, 0));
    expectVec3(multi->getNormal(), Vec3(0, 1, 0));
}

TEST(ConstraintProximityTest, DefaultProximityHandlesNullProximity) {
    ConstraintProximity::SPtr prox = DefaultConstraintProximity::create(nullptr);

    expectVec3(prox->getPosition(), Vec3());
    expectVec3(prox->getVelocity(), Vec3());
    expectVec3(prox->getNormal(), Vec3());

    prox->buildJacobianConstraint(sofa::core::MultiMatrixDerivId(), {}, 1.0, 0);
}

TEST(ConstraintProximityTest, TemplateProximityHandlesNullProximity) {
    TConstraintProximity<MockConstraintProximity>::FUNC_GET_NORMAL normalFunc;
    ConstraintProximity::SPtr prox = TConstraintProximity<MockConstraintProximity>::create(nullptr, normalFunc);

    expectVec3(prox->getPosition(), Vec3());
    expectVec3(prox->getVelocity(), Vec3());
    expectVec3(prox->getNormal(), Vec3());
}
