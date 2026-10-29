#include <ConstraintGeometry/directions/SecondDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {
void registerSecondDirection(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "Computes the constraint direction from the 2nd proximity in pair and a normal handler")
            .add<SecondDirection>());
}
}  // namespace sofa::constraintgeometry
