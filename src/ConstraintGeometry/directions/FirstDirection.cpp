#include <ConstraintGeometry/directions/FirstDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {
void registerFirstDirection(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "Computes the constraint direction from the 1st proximity in pair and a normal handler")
            .add<FirstDirection>());
}
}  // namespace sofa::constraintgeometry
