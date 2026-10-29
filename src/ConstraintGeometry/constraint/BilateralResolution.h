#pragma once

#include <ConstraintGeometry/BaseConstraint.h>
#include <ConstraintGeometry/directions/BindDirection.h>
#include <sofa/core/behavior/ConstraintResolution.h>

#include <cmath>
#include <limits>

namespace sofa {

namespace constraintgeometry {

class BilateralConstraintResolution1 : public sofa::core::behavior::ConstraintResolution {
public:
    BilateralConstraintResolution1(double maxf1 = std::numeric_limits<double>::max(), double comp1 = 0.0)
        : sofa::core::behavior::ConstraintResolution(1) {
        m_maxForce = maxf1;

        m_compliance = comp1;
    }

    virtual void resolution(int line, double** w, double* d, double* force, double* /*dFree*/) override {
        const double denom = w[line][line] + m_compliance;
        if (std::abs(denom) <= std::numeric_limits<double>::epsilon()) return;

        force[line] -= d[line] / denom;
        if (force[line] > m_maxForce) force[line] = m_maxForce;
        if (force[line] < -m_maxForce) force[line] = -m_maxForce;
    }

    double m_compliance;
    double m_maxForce;
};

class BilateralConstraintResolution2 : public sofa::core::behavior::ConstraintResolution {
public:
    BilateralConstraintResolution2(double maxf1 = std::numeric_limits<double>::max(),
                                   double maxf2 = std::numeric_limits<double>::max(), double comp1 = 0.0,
                                   double comp2 = 0.0)
        : sofa::core::behavior::ConstraintResolution(2) {
        m_maxForce[0] = maxf1;
        m_maxForce[1] = maxf2;

        m_compliance[0] = comp1;
        m_compliance[1] = comp2;
    }

    virtual void init(int line, double** w, double* /*force*/) override {
        sofa::type::Mat<2, 2, double> temp;
        for (unsigned j = 0; j < 2; j++) {
            for (unsigned i = 0; i < 2; i++) {
                temp[j][i] = w[line + j][line + i];

                if (i == j) temp[j][i] += m_compliance[i];
            }
        }

        if (!sofa::type::invertMatrix(invW, temp)) {
            // Fallback to a diagonal approximation if the compliance matrix is singular.
            invW = sofa::type::Mat<2, 2, double>();
            for (unsigned i = 0; i < 2; i++) {
                if (std::abs(temp[i][i]) > std::numeric_limits<double>::epsilon()) invW[i][i] = 1.0 / temp[i][i];
            }
        }
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double* /*dFree*/) override {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) force[line + i] -= d[line + j] * invW[i][j];
        }

        for (int i = 0; i < 2; i++) {
            if (force[line + i] > m_maxForce[i])
                force[line + i] = m_maxForce[i];
            else if (force[line + i] < -m_maxForce[i])
                force[line + i] = -m_maxForce[i];
        }
    }

    double m_maxForce[2];
    double m_compliance[2];
    sofa::type::Mat<2, 2, double> invW;
};

class BilateralConstraintResolution3 : public sofa::core::behavior::ConstraintResolution {
public:
    BilateralConstraintResolution3(double maxf1 = std::numeric_limits<double>::max(),
                                   double maxf2 = std::numeric_limits<double>::max(),
                                   double maxf3 = std::numeric_limits<double>::max(), double comp1 = 0.0,
                                   double comp2 = 0.0, double comp3 = 0.0)
        : sofa::core::behavior::ConstraintResolution(3) {
        m_maxForce[0] = maxf1;
        m_maxForce[1] = maxf2;
        m_maxForce[2] = maxf3;

        m_compliance[0] = comp1;
        m_compliance[1] = comp2;
        m_compliance[2] = comp3;
    }

    virtual void init(int line, double** w, double* /*force*/) override {
        sofa::type::Mat<3, 3, double> temp;
        for (unsigned j = 0; j < 3; j++) {
            for (unsigned i = 0; i < 3; i++) {
                temp[j][i] = w[line + j][line + i];

                if (i == j) temp[j][i] += m_compliance[i];
            }
        }

        if (!sofa::type::invertMatrix(invW, temp)) {
            // Fallback to a diagonal approximation if the compliance matrix is singular.
            invW = sofa::type::Mat<3, 3, double>();
            for (unsigned i = 0; i < 3; i++) {
                if (std::abs(temp[i][i]) > std::numeric_limits<double>::epsilon()) invW[i][i] = 1.0 / temp[i][i];
            }
        }
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double* /*dFree*/) override {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) force[line + i] -= d[line + j] * invW[i][j];
        }

        for (int i = 0; i < 3; i++) {
            if (force[line + i] > m_maxForce[i])
                force[line + i] = m_maxForce[i];
            else if (force[line + i] < -m_maxForce[i])
                force[line + i] = -m_maxForce[i];
        }
    }

    double m_maxForce[3];
    double m_compliance[3];
    sofa::type::Mat<3, 3, double> invW;
};

}  // namespace constraintgeometry

}  // namespace sofa
