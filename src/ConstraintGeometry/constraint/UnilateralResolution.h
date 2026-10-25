#pragma once

#include <ConstraintGeometry/BaseConstraint.h>
#include <ConstraintGeometry/directions/ContactDirection.h>
#include <sofa/core/behavior/ConstraintResolution.h>

#include <cmath>
#include <limits>

namespace sofa {

namespace constraintgeometry {

/*!
 * \brief The UnilateralConstraintResolution class solves unilateral constraints
 */
class UnilateralConstraintResolution : public sofa::core::behavior::ConstraintResolution {
public:
    /*!
     * \brief UnilateralConstraintResolution Constructor
     * \param m : double, maxForce value
     */
    UnilateralConstraintResolution(double m = std::numeric_limits<double>::max())
        : sofa::core::behavior::ConstraintResolution(1), m_maxForce(m) {
    }

    /*!
     * \brief resolution : updates and keeps the force applied on 'line'
     * in the interval [0, maxForce]
     * \param line
     * \param w
     * \param d
     * \param force
     */
    virtual void resolution(int line, double** w, double* d, double* force, double* /*dFree*/) override {
        const double denom = w[line][line];
        if (std::abs(denom) <= std::numeric_limits<double>::epsilon()) return;

        force[line] -= d[line] / denom;

        if (force[line] > m_maxForce)
            force[line] = m_maxForce;
        else if (force[line] < 0)
            force[line] = 0.0;
    }

    double m_maxForce;
};

/*!
 * \brief The UnilateralFrictionResolution class
 * Solves unilateral friction constraints
 */
class UnilateralFrictionResolution : public sofa::core::behavior::ConstraintResolution {
public:
    UnilateralFrictionResolution(double f, double m0 = std::numeric_limits<double>::max(),
                                 double m1 = std::numeric_limits<double>::max(),
                                 double m2 = std::numeric_limits<double>::max())
        : sofa::core::behavior::ConstraintResolution(3),
          m_maxForce0(m0),
          m_maxForce1(m1),
          m_maxForce2(m2),
          m_friction(f) {
    }

    virtual void resolution(int line, double** w, double* d, double* force, double* /*dFree*/) override {
        constexpr double epsilon = std::numeric_limits<double>::epsilon();

        // we solve the first constraint as if there was no friction
        if (std::abs(w[line][line]) <= epsilon) return;

        const double inv0 = 1.0 / w[line][line];
        const double corr0 = -inv0 * d[line + 0];
        force[line + 0] += corr0;

        // we check unilateral conditions
        if (force[line + 0] < 0) {
            force[line + 0] = 0.0;
            force[line + 1] = 0.0;
            force[line + 2] = 0.0;
            return;
        }

        // we clamp the force is needed
        if (force[line + 0] > m_maxForce0) force[line + 0] = m_maxForce0;

        // now we compute tangential forces
        double corr1 = 0.0;
        if (std::abs(w[line + 1][line + 1]) > epsilon) {
            d[line + 1] += w[line + 1][line + 0] * corr0;
            corr1 = -d[line + 1] / w[line + 1][line + 1];
            force[line + 1] += corr1;
        }

        if (std::abs(w[line + 2][line + 2]) > epsilon) {
            d[line + 2] += w[line + 2][line + 0] * corr0 + w[line + 2][line + 1] * corr1;
            const double corr2 = -d[line + 2] / w[line + 2][line + 2];
            force[line + 2] += corr2;
        }

        const double slip_force = force[line + 0] * m_friction;

        if (force[line + 1] > slip_force)
            force[line + 1] = slip_force;
        else if (force[line + 1] < -slip_force)
            force[line + 1] = -slip_force;

        if (force[line + 2] > slip_force)
            force[line + 2] = slip_force;
        else if (force[line + 2] < -slip_force)
            force[line + 2] = -slip_force;

        if (force[line + 1] > m_maxForce1)
            force[line + 1] = m_maxForce1;
        else if (force[line + 1] < -m_maxForce1)
            force[line + 1] = -m_maxForce1;

        if (force[line + 2] > m_maxForce2)
            force[line + 2] = m_maxForce2;
        else if (force[line + 2] < -m_maxForce2)
            force[line + 2] = -m_maxForce2;
    }

    double m_maxForce0, m_maxForce1, m_maxForce2;
    double m_friction;
};

}  // namespace constraintgeometry

}  // namespace sofa
