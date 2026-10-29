#include <ConstraintGeometry/directions/TransverseDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {
void registerTransverseDirection(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "Two constraint directions orthogonal to the axis given by a normal handler (sliding along the axis)")
            .add<TransverseDirection>());
}
}  // namespace sofa::constraintgeometry
