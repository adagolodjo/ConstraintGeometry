#pragma once

#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/proximity/MechanicalProximity.h>
#include <CollisionAlgorithm/proximity/TriangleProximity.h>
#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintProximity.h>

#include <cmath>
#include <limits>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API GouraudTriangleNormalHandler : public BaseNormalHandler {
public:
    SOFA_CLASS(GouraudTriangleNormalHandler, BaseNormalHandler);

    core::objectmodel::SingleLink<GouraudTriangleNormalHandler, BaseGeometry,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_geometry;

    GouraudTriangleNormalHandler() : l_geometry(initLink("geometry", "link to the second normal handler")) {
        l_geometry.setPath("@.");
    }

    BaseGeometry* getGeometry() override {
        return l_geometry.get();
    }

    void prepareDetection() override {
    }

    const std::type_info& getTypeInfo() override {
        return typeid(GouraudTriangleNormalHandler);
    }

    template <class PROXIMITY>
    type::Vec3 getNormal(const typename PROXIMITY::SPtr& prox);

    template <class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(GouraudTriangleNormalHandler* handler,
                                                              typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(
            prox, [handler](const typename PROXIMITY::SPtr& p) { return handler->getNormal<PROXIMITY>(p); });
    }
};

template <>
inline type::Vec3 GouraudTriangleNormalHandler::getNormal<collisionalgorithm::TriangleProximity>(
    const collisionalgorithm::TriangleProximity::SPtr& prox) {
    if (prox == nullptr || prox->element() == nullptr) return type::Vec3();
    return prox->element()->getTriangleInfo().N;
}

template <>
inline type::Vec3
GouraudTriangleNormalHandler::getNormal<collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>(
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
