#pragma once

#include <ConstraintGeometry/BaseNormalHandler.h>
#include <ConstraintGeometry/ConstraintDirection.h>

#include <cmath>
#include <limits>

namespace sofa::constraintgeometry {

/*!
 * \brief The ContactDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class ContactDirection : public ConstraintDirection {
public:
    SOFA_CLASS(ContactDirection, ConstraintDirection);

    core::objectmodel::SingleLink<ContactDirection, BaseNormalHandler,
                                  BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_handler;

    ContactDirection() : l_handler(initLink("handler", "link to the second normal handler")) {
    }

    void init() override {
        if (l_handler == NULL) {
            std::cerr << "Cannot find normal handler in ContactDirection " << std::endl;
        }
    }

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const BaseProximity::SPtr& first,
                                             const BaseProximity::SPtr& second) const override {
        if (l_handler == NULL || first == nullptr || second == nullptr) return ConstraintNormal();

        const type::Vec3 delta = first->getPosition() - second->getPosition();
        type::Vec3 mainDir = delta.normalized();
        if (mainDir.norm() <= std::numeric_limits<double>::epsilon()) mainDir = type::Vec3();

        const ConstraintProximity::SPtr& cp = l_handler->createConstraintProximity(second);
        if (cp == NULL) {
            std::cerr << "Error cannot create normal in ContactDirection " << std::endl;
            return ConstraintNormal();
        }

        type::Vec3 secondDir = -cp->getNormal();
        if (mainDir.norm() <= std::numeric_limits<double>::epsilon()) {
            mainDir = secondDir;
        } else if (dot(mainDir, secondDir) <= std::numeric_limits<double>::epsilon()) {
            mainDir = secondDir;
        }

        return ConstraintNormal(mainDir);
    }
};

}  // namespace sofa::constraintgeometry
