#pragma once

#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintDirection.h>

namespace sofa::constraintgeometry {

/*!
 * \brief The TransverseDirection class
 * Two directions orthogonal to the axis given by a normal handler (for instance the EdgeNormalHandler of a guide, whose
 * "normal" is the direction of the edge).  With a ConstraintBilateral it keeps a point on the axis and lets it slide along it,
 * with two lines of constraint instead of the three of ConstraintInsertion.
 */
class TransverseDirection : public ConstraintDirection {
public:
    SOFA_CLASS(TransverseDirection, ConstraintDirection);

    core::objectmodel::SingleLink<TransverseDirection, BaseNormalHandler, BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_handler;
    Data<bool> d_useFirst;

    TransverseDirection()
        : l_handler(initLink("handler", "link to the normal handler that gives the axis")),
          d_useFirst(initData(&d_useFirst, false, "useFirst",
                              "Take the axis from the first proximity of the pair (default: from the second)")) {
    }

    void init() override {
        if (l_handler == NULL) {
            std::cerr << "Cannot find normal handler in TransverseDirection " << std::endl;
        }
    }

    ConstraintNormal createConstraintsNormal(const BaseProximity::SPtr& first,
                                             const BaseProximity::SPtr& second) const override {
        const BaseProximity::SPtr& source = d_useFirst.getValue() ? first : second;
        if (l_handler == NULL || source == nullptr) return ConstraintNormal();

        const ConstraintProximity::SPtr cp = l_handler->createConstraintProximity(source);
        if (cp == NULL) {
            std::cerr << "Error cannot create normal in TransverseDirection " << std::endl;
            return ConstraintNormal();
        }
        return ConstraintNormal::transverse(cp->getNormal());
    }
};

}  // namespace sofa::constraintgeometry
