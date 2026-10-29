#pragma once

#include <ConstraintGeometry/BaseConstraint.h>
#include <ConstraintGeometry/constraint/BilateralResolution.h>
#include <ConstraintGeometry/directions/BindDirection.h>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API ConstraintBilateral
    : public TBaseConstraint<collisionalgorithm::BaseProximity, collisionalgorithm::BaseProximity> {
public:
    SOFA_CLASS(ConstraintBilateral, SOFA_TEMPLATE2(TBaseConstraint, BaseProximity, BaseProximity));

    Data<sofa::type::vector<double> > d_maxForce;
    Data<sofa::type::vector<double> > d_compliance;
    core::objectmodel::SingleLink<ConstraintBilateral, ConstraintDirection,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_directions;

    ConstraintBilateral()
        : d_maxForce(initData(&d_maxForce, "maxForce", "Max force")),
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

    virtual ConstraintNormal createConstraintNormal(const BaseProximity::SPtr& first,
                                                    const BaseProximity::SPtr& second) const override {
        if (l_directions == NULL) return ConstraintNormal();
        return l_directions->createConstraintsNormal(first, second);
    }

    core::behavior::ConstraintResolution* createConstraintResolution(const BaseInternalConstraint* cst) const override {
        if (cst->size() == 1) {
            double maxf = std::numeric_limits<double>::max();
            double comp = 0.0;
            if (d_maxForce.getValue().size() > 0) maxf = d_maxForce.getValue()[0];
            if (d_compliance.getValue().size() > 0) comp = d_compliance.getValue()[0];
            return new BilateralConstraintResolution1(maxf, comp);
        } else if (cst->size() == 2) {
            double maxf[2] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
            double comp[2] = {0.0, 0.0};

            if (d_maxForce.getValue().size() > 0) maxf[0] = d_maxForce.getValue()[0];
            if (d_maxForce.getValue().size() > 1) maxf[1] = d_maxForce.getValue()[1];

            if (d_compliance.getValue().size() > 0) comp[0] = d_compliance.getValue()[0];
            if (d_compliance.getValue().size() > 1) comp[1] = d_compliance.getValue()[1];

            return new BilateralConstraintResolution2(maxf[0], maxf[1], comp[0], comp[1]);
        } else if (cst->size() == 3) {
            double maxf[3] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max()};
            double comp[3] = {0.0, 0.0, 0.0};

            if (d_maxForce.getValue().size() > 0) maxf[0] = d_maxForce.getValue()[0];
            if (d_maxForce.getValue().size() > 1) maxf[1] = d_maxForce.getValue()[1];
            if (d_maxForce.getValue().size() > 2) maxf[2] = d_maxForce.getValue()[2];

            if (d_compliance.getValue().size() > 0) comp[0] = d_compliance.getValue()[0];
            if (d_compliance.getValue().size() > 1) comp[1] = d_compliance.getValue()[1];
            if (d_compliance.getValue().size() > 2) comp[2] = d_compliance.getValue()[2];

            return new BilateralConstraintResolution3(maxf[0], maxf[1], maxf[2], comp[0], comp[1], comp[2]);
        }

        std::cerr << "Error the size of the constraint is not correct in ConstraintBilateral size=" << cst->size()
                  << std::endl;
        return NULL;
    }

protected:
    sofa::type::vector<std::string> getBaseConstraintIdentifiers() override final {
        return {"Bilateral"};
    }
};

}  // namespace sofa::constraintgeometry
