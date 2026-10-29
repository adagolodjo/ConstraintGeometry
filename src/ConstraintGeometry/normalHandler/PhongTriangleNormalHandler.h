#pragma once

#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/proximity/MechanicalProximity.h>
#include <CollisionAlgorithm/proximity/TriangleProximity.h>
#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintProximity.h>

#include <cmath>
#include <limits>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API PhongTriangleNormalHandler : public BaseNormalHandler {
public:
    SOFA_CLASS(PhongTriangleNormalHandler, BaseNormalHandler);

    core::objectmodel::SingleLink<PhongTriangleNormalHandler, BaseGeometry,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_geometry;

    PhongTriangleNormalHandler() : l_geometry(initLink("geometry", "link to the second normal handler")) {
        l_geometry.setPath("@.");
    }

    BaseGeometry* getGeometry() override {
        return l_geometry.get();
    }

    void prepareDetection() override {
    }

    const std::type_info& getTypeInfo() override {
        return typeid(PhongTriangleNormalHandler);
    }

    template <class PROXIMITY>
    type::Vec3 getNormal(const typename PROXIMITY::SPtr& prox);

    template <class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(PhongTriangleNormalHandler* handler,
                                                              typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(
            prox, [handler](const typename PROXIMITY::SPtr& p) { return handler->getNormal<PROXIMITY>(p); });
    }
};

template <>
inline type::Vec3 PhongTriangleNormalHandler::getNormal<collisionalgorithm::TriangleProximity>(
    const collisionalgorithm::TriangleProximity::SPtr& prox) {
    if (prox == nullptr || prox->element() == nullptr) return type::Vec3();

    auto element = prox->element();
    const auto& points = element->pointElements();
    if (points.size() < 3 || points[0] == nullptr || points[1] == nullptr || points[2] == nullptr) return type::Vec3();

    const collisionalgorithm::BaseProximity::SPtr& p0 = points[0]->getP0();
    const collisionalgorithm::BaseProximity::SPtr& p1 = points[1]->getP0();
    const collisionalgorithm::BaseProximity::SPtr& p2 = points[2]->getP0();
    if (p0 == nullptr || p1 == nullptr || p2 == nullptr) return type::Vec3();

    ConstraintProximityOperation::FUNC operation = ConstraintProximityOperation::get(getTypeInfo(), p0->getTypeInfo());

    const ConstraintProximity::SPtr cp0 = operation(this, p0);
    const ConstraintProximity::SPtr cp1 = operation(this, p1);
    const ConstraintProximity::SPtr cp2 = operation(this, p2);
    if (cp0 == nullptr || cp1 == nullptr || cp2 == nullptr) return type::Vec3();

    type::Vec3 N0_point = cp0->getNormal();
    type::Vec3 N1_point = cp1->getNormal();
    type::Vec3 N2_point = cp2->getNormal();

    type::Vec3 N =
        N0_point.normalized() * prox->f0() + N1_point.normalized() * prox->f1() + N2_point.normalized() * prox->f2();

    if (N.norm() <= std::numeric_limits<double>::epsilon()) return type::Vec3();
    return N.normalized();
}

template <>
inline type::Vec3
PhongTriangleNormalHandler::getNormal<collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >(
    const collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>::SPtr& prox) {
    if (prox == nullptr || prox->getGeometry() == nullptr) return type::Vec3();

    const auto& points = prox->getGeometry()->pointElements();
    if (prox->getPId() >= points.size()) return type::Vec3();

    const collisionalgorithm::PointElement::SPtr& element = points[prox->getPId()];
    if (element == nullptr) return type::Vec3();

    type::Vec3 N0_point;
    for (auto it = element->triangleAround().cbegin(); it != element->triangleAround().cend(); it++) {
        if (*it != nullptr) N0_point += (*it)->getTriangleInfo().N;
    }

    if (N0_point.norm() <= std::numeric_limits<double>::epsilon()) return type::Vec3();
    return N0_point.normalized();
}

}  // namespace sofa::constraintgeometry
