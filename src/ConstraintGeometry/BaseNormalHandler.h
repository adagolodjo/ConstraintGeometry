#pragma once

#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/CollisionPipeline.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>
#include <ConstraintGeometry/ConstraintProximity.h>
#include <ConstraintGeometry/config.h>
#include <ConstraintGeometry/operations/ConstraintProximityOperation.h>

namespace sofa ::constraintgeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class SOFA_CONSTRAINTGEOMETRY_API BaseNormalHandler : public collisionalgorithm::CollisionComponent {
public:
    SOFA_ABSTRACT_CLASS(BaseNormalHandler, collisionalgorithm::CollisionComponent);

    typedef collisionalgorithm::BaseGeometry BaseGeometry;
    typedef collisionalgorithm::BaseProximity BaseProximity;

    void init() override {
        if (getGeometry() == NULL) {
            std::cerr << "Error cannot find the geometry" << std::endl;
            return;
        }

        getGeometry()->addSlave(this);
    }

    virtual BaseGeometry* getGeometry() = 0;

    virtual const std::type_info& getTypeInfo() = 0;

    virtual ConstraintProximity::SPtr createConstraintProximity(const BaseProximity::SPtr& prox) {
        if (prox == nullptr) {
            msg_error(this) << "Cannot create a constraint proximity from a null proximity";
            return nullptr;
        }

        ConstraintProximityOperation::FUNC operation =
            ConstraintProximityOperation::get(getTypeInfo(), prox->getTypeInfo());
        ConstraintProximity::SPtr constraintProx = operation(this, prox);
        if (constraintProx == nullptr) {
            msg_error(this) << "No constraint proximity registered for the required proximity type";
        }

        return constraintProx;
    }

    void draw(const core::visual::VisualParams* vparams) override {
        if (!vparams->displayFlags().getShowNormals()) return;
        if (getGeometry() == nullptr) return;

        type::RGBAColor color(1, 0, 0, 1);
        double scale = 1.0;

        glDisable(GL_LIGHTING);
        if (color[3] == 0.0) return;

        collisionalgorithm::Operations::CreateCenterProximity::FUNC operation =
            collisionalgorithm::Operations::CreateCenterProximity::Operation::get(getGeometry()->pointBegin());

        for (auto it = getGeometry()->pointBegin(); it != getGeometry()->end(); it++) {
            auto prox = operation(it->element());
            auto cp = createConstraintProximity(prox);
            if (cp == nullptr) continue;

            vparams->drawTool()->drawArrow(cp->getPosition(), cp->getPosition() - cp->getNormal() * scale, scale * 0.1,
                                           color);
        }
    }
};

}  // namespace sofa::constraintgeometry
