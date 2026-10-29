#include <ConstraintGeometry/normalHandler/PhongTriangleNormalHandler.h>
#include <ConstraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {

void registerPhongTriangleNormalHandler(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(sofa::core::ObjectRegistrationData("").add<PhongTriangleNormalHandler>());
}

int phong_reg_tri =
    ConstraintProximityOperation::register_func<PhongTriangleNormalHandler, collisionalgorithm::TriangleProximity>(
        &PhongTriangleNormalHandler::buildCstProximity<collisionalgorithm::TriangleProximity>);

int phong_reg_mech =
    ConstraintProximityOperation::register_func<PhongTriangleNormalHandler,
                                                collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>(
        &PhongTriangleNormalHandler::buildCstProximity<
            collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>);

}  // namespace sofa::constraintgeometry
