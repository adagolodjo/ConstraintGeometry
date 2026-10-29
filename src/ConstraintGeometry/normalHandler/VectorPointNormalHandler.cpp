#include <ConstraintGeometry/normalHandler/VectorPointNormalHandler.h>
#include <ConstraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {
void registerVectorPointNormalHandler(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(sofa::core::ObjectRegistrationData("").add<VectorPointNormalHandler>());
}

int vectorPoint_reg = ConstraintProximityOperation::register_func<
    VectorPointNormalHandler, collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >(
    &VectorPointNormalHandler::buildCstProximity<
        collisionalgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >);
}  // namespace sofa::constraintgeometry
