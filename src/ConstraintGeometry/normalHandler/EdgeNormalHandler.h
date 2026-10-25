#pragma once

#include <CollisionAlgorithm/geometry/EdgeGeometry.h>
#include <CollisionAlgorithm/proximity/EdgeProximity.h>
#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintProximity.h>

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
            prox, std::bind(&EdgeNormalHandler::getNormal<PROXIMITY>, handler, std::placeholders::_1));
    }
};

template <>
type::Vec3 EdgeNormalHandler::getNormal<collisionalgorithm::EdgeProximity>(
    const collisionalgorithm::EdgeProximity::SPtr& prox) {
    return (prox->element()->getP1()->getPosition() - prox->element()->getP0()->getPosition()).normalized();
}

}  // namespace sofa::constraintgeometry
