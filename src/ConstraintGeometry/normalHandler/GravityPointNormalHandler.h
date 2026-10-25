#pragma once

#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/proximity/PointProximity.h>
#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintProximity.h>

#include <cmath>
#include <limits>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API GravityPointNormalHandler : public BaseNormalHandler {
public:
    SOFA_CLASS(GravityPointNormalHandler, BaseNormalHandler);

    core::objectmodel::SingleLink<GravityPointNormalHandler, collisionalgorithm::BaseGeometry,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_geometry;

    GravityPointNormalHandler() : l_geometry(initLink("geometry", "Link to TriangleGeometry")) {
        l_geometry.setPath("@.");
    }

    BaseGeometry* getGeometry() override {
        return l_geometry.get();
    }

    void prepareDetection() override {
        m_gcenter = type::Vec3();
        if (l_geometry == nullptr) return;

        unsigned nbPoints = 0;
        for (auto it = l_geometry->pointBegin(); it != l_geometry->end(); it++) {
            auto point = std::dynamic_pointer_cast<collisionalgorithm::PointElement>(it->element());
            if (point == nullptr || point->getP0() == nullptr) continue;

            m_gcenter += point->getP0()->getPosition();
            nbPoints++;
        }

        if (nbPoints) m_gcenter *= 1.0 / nbPoints;
    }

    template <class PROXIMITY>
    type::Vec3 getNormal(const typename PROXIMITY::SPtr& prox);

    const std::type_info& getTypeInfo() override {
        return typeid(GravityPointNormalHandler);
    }

    template <class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(GravityPointNormalHandler* handler,
                                                              typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(
            prox, [handler](const typename PROXIMITY::SPtr& p) { return handler->getNormal<PROXIMITY>(p); });
    }

private:
    type::Vec3 m_gcenter;
};

template <>
inline type::Vec3 GravityPointNormalHandler::getNormal<collisionalgorithm::PointProximity>(
    const collisionalgorithm::PointProximity::SPtr& prox) {
    if (prox == nullptr) return type::Vec3();

    const type::Vec3 direction = prox->getPosition() - m_gcenter;
    if (direction.norm() <= std::numeric_limits<double>::epsilon()) return type::Vec3();

    return direction.normalized();
}

}  // namespace sofa::constraintgeometry
