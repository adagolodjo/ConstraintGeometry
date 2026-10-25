#include <ConstraintGeometry/directions/ContactDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {
void registerContactDirection(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("Computes the constraint direction given proximities and a normal handler")
            .add<ContactDirection>());
}
}  // namespace sofa::constraintgeometry
