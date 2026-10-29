#pragma once

#include <ConstraintGeometry/ConstraintDirection.h>

namespace sofa::constraintgeometry {

/*!
 * \brief The BindDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class BindDirection : public ConstraintDirection {
public:
    SOFA_CLASS(BindDirection, ConstraintDirection);

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const BaseProximity::SPtr& first,
                                             const BaseProximity::SPtr& second) const override {
        if (first == nullptr || second == nullptr) return ConstraintNormal();
        return ConstraintNormal(first->getPosition() - second->getPosition());
    }
};

}  // namespace sofa::constraintgeometry
