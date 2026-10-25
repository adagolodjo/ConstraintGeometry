#include <ConstraintGeometry/constraint/ConstraintBilateral.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {
void registerConstraintBilateral(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(sofa::core::ObjectRegistrationData(
                                 "Implements bilateral constraints between an origin and a destination geometry")
                                 .add<ConstraintBilateral>());
}
}  // namespace sofa::constraintgeometry
