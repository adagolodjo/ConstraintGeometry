#include <ConstraintGeometry/constraint/ConstraintUnilateral.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {
void registerConstraintUnilateral(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(sofa::core::ObjectRegistrationData(
                                 "Implements unilateral constraints between an origin and a destination geometry")
                                 .add<ConstraintUnilateral>());
}
}  // namespace sofa::constraintgeometry
