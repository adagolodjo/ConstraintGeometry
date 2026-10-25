#include <ConstraintGeometry/normalHandler/GouraudTriangleNormalHandler.h>
#include <ConstraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {

void registerGouraudTriangleNormalHandler(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(sofa::core::ObjectRegistrationData("").add<GouraudTriangleNormalHandler>());
}

int gouraud_reg_tri =
    ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler, collisionalgorithm::TriangleProximity>(
        &GouraudTriangleNormalHandler::buildCstProximity<collisionalgorithm::TriangleProximity>);

int gouraud_reg_mech =
    ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler,
                                                collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>(
        &GouraudTriangleNormalHandler::buildCstProximity<
            collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>);

}  // namespace sofa::constraintgeometry
