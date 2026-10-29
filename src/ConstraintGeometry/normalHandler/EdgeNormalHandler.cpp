#include <ConstraintGeometry/normalHandler/EdgeNormalHandler.h>
#include <ConstraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintgeometry {

void registerEdgeNormalHandler(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(sofa::core::ObjectRegistrationData("Default implementation of normal computations return "
                                                                "normal in the direction of the edge")
                                 .add<EdgeNormalHandler>());
}

int edge_reg = ConstraintProximityOperation::register_func<EdgeNormalHandler, collisionalgorithm::EdgeProximity>(
    &EdgeNormalHandler::buildCstProximity<collisionalgorithm::EdgeProximity>);

}  // namespace sofa::constraintgeometry
