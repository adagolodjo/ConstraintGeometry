#include <ConstraintGeometry/normalHandler/GravityPointNormalHandler.h>
#include <ConstraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {

void registerGravityPointNormalHandler(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("return the normal between the gravity center of the object and each point")
            .add<GravityPointNormalHandler>());
}

int gravityPoint_reg =
    ConstraintProximityOperation::register_func<GravityPointNormalHandler, collisionalgorithm::PointProximity>(
        &GravityPointNormalHandler::buildCstProximity<collisionalgorithm::PointProximity>);

}  // namespace sofa::constraintgeometry
