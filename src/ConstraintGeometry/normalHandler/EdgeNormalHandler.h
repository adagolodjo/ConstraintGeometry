#pragma once

#include <CollisionAlgorithm/geometry/EdgeGeometry.h>
#include <CollisionAlgorithm/proximity/EdgeProximity.h>
#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintProximity.h>

#include <cmath>
#include <limits>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API EdgeNormalHandler : public BaseNormalHandler {
public:
    SOFA_CLASS(EdgeNormalHandler, BaseNormalHandler);

    core::objectmodel::SingleLink<EdgeNormalHandler, BaseGeometry, BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_geometry;

    EdgeNormalHandler() : l_geometry(initLink("geometry", "link to the second normal handler")) {
        l_geometry.setPath("@.");
    }

    BaseGeometry* getGeometry() override {
        return l_geometry.get();
    }

    void prepareDetection() override {
    }

    template <class PROXIMITY>
    type::Vec3 getNormal(const typename PROXIMITY::SPtr& prox);

    const std::type_info& getTypeInfo() override {
        return typeid(EdgeNormalHandler);
    }

    template <class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(EdgeNormalHandler* handler,
                                                              typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(
            prox, [handler](const typename PROXIMITY::SPtr& p) { return handler->getNormal<PROXIMITY>(p); });
    }
};

template <>
inline type::Vec3 EdgeNormalHandler::getNormal<collisionalgorithm::EdgeProximity>(
    const collisionalgorithm::EdgeProximity::SPtr& prox) {
    if (prox == nullptr || prox->element() == nullptr) return type::Vec3();

    const auto element = prox->element();
    if (element->getP0() == nullptr || element->getP1() == nullptr) return type::Vec3();

    const type::Vec3 direction = element->getP1()->getPosition() - element->getP0()->getPosition();
    if (direction.norm() <= std::numeric_limits<double>::epsilon()) return type::Vec3();

    return direction.normalized();
}

}  // namespace sofa::constraintgeometry
