#include <ConstraintGeometry/directions/BindDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {
void registerBindDirection(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(sofa::core::ObjectRegistrationData(
                                 "Computes the constraint direction given proximities from the 2nd to the 1st point")
                                 .add<BindDirection>());
}
}  // namespace sofa::constraintgeometry
