#pragma once

#include <ConstraintGeometry/BaseConstraint.h>
#include <ConstraintGeometry/constraint/InsertionResolution.h>
#include <ConstraintGeometry/directions/BindDirection.h>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API ConstraintInsertion
    : public TBaseConstraint<collisionalgorithm::BaseProximity, collisionalgorithm::BaseProximity> {
public:
    SOFA_CLASS(ConstraintInsertion, SOFA_TEMPLATE2(TBaseConstraint, BaseProximity, BaseProximity));

    Data<SReal> d_frictionCoeff;
    Data<sofa::type::vector<double> > d_maxForce;
    Data<sofa::type::vector<double> > d_compliance;
    core::objectmodel::SingleLink<ConstraintInsertion, ConstraintDirection,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_directions;

    ConstraintInsertion()
        : d_frictionCoeff(
              initData(&d_frictionCoeff, 0.0, "frictionCoeff", "static friction coefficient (should be less than 1)")),
          d_maxForce(initData(&d_maxForce, "maxForce", "Max force")),
          d_compliance(initData(&d_compliance, "compliance", "Max force")),
          l_directions(initLink("directions", "link to the default direction")) {
    }

    void init() override {  // make sure we have a direction
        BaseConstraint::init();
        if (l_directions == NULL) {
            msg_error(this) << "Error you must provide the directions to solve the constraints ";
        } else
            this->addSlave(l_directions.get());
    }

    ConstraintNormal createConstraintNormal(const BaseProximity::SPtr& first,
                                            const BaseProximity::SPtr& second) const override {
        if (l_directions == NULL) return ConstraintNormal();

        ConstraintNormal constraintNormal = l_directions->createConstraintsNormal(first, second);
        if (constraintNormal.size() == 0) return constraintNormal;

        while (constraintNormal.size() < 3) {
            const unsigned previousSize = constraintNormal.size();
            constraintNormal.addOrthogonalDirection();
            if (constraintNormal.size() == previousSize) return ConstraintNormal();
        }

        return constraintNormal;
    }

    core::behavior::ConstraintResolution* createConstraintResolution(const BaseInternalConstraint* cst) const override {
        if (cst->size() != 3) {
            std::cerr << "Error the size of the constraint is not correct in ConstraintInsertion size=" << cst->size()
                      << std::endl;
            return NULL;
        }

        double maxf[3] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max()};
        double comp[3] = {0.0, 0.0, 0.0};

        if (d_maxForce.getValue().size() > 0) maxf[0] = d_maxForce.getValue()[0];
        if (d_maxForce.getValue().size() > 1) maxf[1] = d_maxForce.getValue()[1];
        if (d_maxForce.getValue().size() > 2) maxf[2] = d_maxForce.getValue()[2];

        if (d_compliance.getValue().size() > 0) comp[0] = d_compliance.getValue()[0];
        if (d_compliance.getValue().size() > 1) comp[1] = d_compliance.getValue()[1];
        if (d_compliance.getValue().size() > 2) comp[2] = d_compliance.getValue()[2];

        return new InsertionConstraintResolution(d_frictionCoeff.getValue(), maxf[0], maxf[1], maxf[2], comp[0],
                                                 comp[1], comp[2]);
    }

protected:
    sofa::type::vector<std::string> getBaseConstraintIdentifiers() override final {
        return {"Insertion"};
    }
};

}  // namespace sofa::constraintgeometry
