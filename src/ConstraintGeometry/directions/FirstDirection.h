#pragma once

#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintDirection.h>

namespace sofa::constraintgeometry {

/*!
 * \brief The FirstDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class FirstDirection : public ConstraintDirection {
public:
    SOFA_CLASS(FirstDirection, ConstraintDirection);

    core::objectmodel::SingleLink<FirstDirection, BaseNormalHandler,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_handler;

    FirstDirection() : l_handler(initLink("handler", "link to the second normal handler")) {
    }

    void init() override {
        if (l_handler == NULL) {
            std::cerr << "Cannot find normal handler in FirstDirection " << std::endl;
        }
    }

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const BaseProximity::SPtr& first,
                                             const BaseProximity::SPtr&) const override {
        if (l_handler == NULL) return ConstraintNormal();

        const ConstraintProximity::SPtr& cp = l_handler->createConstraintProximity(first);
        if (cp == NULL) {
            std::cerr << "Error cannot create normal in ContactDirection " << std::endl;
            return ConstraintNormal();
        }

        return ConstraintNormal(cp->getNormal());
    }
};

}  // namespace sofa::constraintgeometry
