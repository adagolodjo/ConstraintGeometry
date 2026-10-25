#pragma once

#include <CollisionAlgorithm/geometry/PointGeometry.h>
#include <CollisionAlgorithm/proximity/MechanicalProximity.h>
#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintProximity.h>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API VectorPointNormalHandler : public BaseNormalHandler {
public:
    SOFA_CLASS(VectorPointNormalHandler, BaseNormalHandler);

    Data<type::vector<type::Vec3>> d_normals;
    core::objectmodel::SingleLink<VectorPointNormalHandler, collisionalgorithm::BaseGeometry,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_geometry;

    VectorPointNormalHandler()
        : d_normals(initData(&d_normals, "normals", "Vector of normals")),
          l_geometry(initLink("geometry", "link to the second normal handler")) {
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
        return typeid(VectorPointNormalHandler);
    }

    template <class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(VectorPointNormalHandler* handler,
                                                              typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(
            prox, std::bind(&VectorPointNormalHandler::getNormal<PROXIMITY>, handler, std::placeholders::_1));
    }
};

template <>
inline type::Vec3
VectorPointNormalHandler::getNormal<collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>(
    const collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>::SPtr& prox) {
    type::Vec3 N;

    if (prox->getPId() >= d_normals.getValue().size()) {
        std::cerr << "Wrong id in VectorPointNormalHandler" << std::endl;
        return N;
    }

    N = d_normals.getValue()[prox->getPId()];
    return N;
}

}  // namespace sofa::constraintgeometry
