#pragma once

#include <CollisionAlgorithm/proximity/MechanicalProximity.h>
#include <CollisionAlgorithm/proximity/TriangleProximity.h>
#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintProximity.h>

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
            prox, std::bind(&GouraudTriangleNormalHandler::getNormal<PROXIMITY>, handler, std::placeholders::_1));
    }
};

template <>
inline type::Vec3 GouraudTriangleNormalHandler::getNormal<collisionalgorithm::TriangleProximity>(
    const collisionalgorithm::TriangleProximity::SPtr& prox) {
    return prox->element()->getTriangleInfo().N;
}

template <>
inline type::Vec3
GouraudTriangleNormalHandler::getNormal<collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>(
    const collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>::SPtr& prox) {
    const collisionalgorithm::PointElement::SPtr& element = prox->getGeometry()->pointElements()[prox->getPId()];

    type::Vec3 N0_point;
    for (auto it = element->triangleAround().cbegin(); it != element->triangleAround().cend(); it++) {
        N0_point += (*it)->getTriangleInfo().N;
        break;
    }

    return N0_point.normalized();
}

}  // namespace sofa::constraintgeometry
