#pragma once

#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintDirection.h>

namespace sofa::constraintgeometry {

/*!
 * \brief The SecondDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class SecondDirection : public ConstraintDirection {
public:
    SOFA_CLASS(SecondDirection, ConstraintDirection);

    core::objectmodel::SingleLink<SecondDirection, BaseNormalHandler,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_handler;

    SecondDirection() : l_handler(initLink("handler", "link to the second normal handler")) {
    }

    void init() override {
        if (l_handler == NULL) {
            std::cerr << "Cannot find normal handler in SecondDirection " << std::endl;
        }
    }

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const BaseProximity::SPtr&,
                                             const BaseProximity::SPtr& second) const override {
        if (l_handler == NULL || second == nullptr) return ConstraintNormal();

        const ConstraintProximity::SPtr& cp = l_handler->createConstraintProximity(second);
        if (cp == NULL) {
            std::cerr << "Error cannot create normal in SecondDirection " << std::endl;
            return ConstraintNormal();
        }

        return ConstraintNormal(-cp->getNormal());
    }
};

}  // namespace sofa::constraintgeometry
