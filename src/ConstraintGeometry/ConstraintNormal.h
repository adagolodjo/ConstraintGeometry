#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <ConstraintGeometry/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/type/vector.h>

namespace sofa::constraintgeometry {

/*!
 * \brief The ConstraintNormal class
 */

class SOFA_CONSTRAINTGEOMETRY_API ConstraintNormal {
public:
    typedef collisionalgorithm::BaseProximity BaseProximity;
    typedef collisionalgorithm::BaseBaseProximity FIRST;
    typedef collisionalgorithm::BaseBaseProximity SECOND;
    typedef std::function<double(const typename FIRST::SPtr &, const typename SECOND::SPtr &, const type::Vec3 &)>
        ViolationFunction;

    static double defaultViolationFunction(const typename FIRST::SPtr &first, const typename SECOND::SPtr &second,
                                           const type::Vec3 &normal) {
        const type::Vec3 &PFree = first->getPosition(core::vec_id::write_access::freePosition);
        const type::Vec3 &QFree = second->getPosition(core::vec_id::write_access::freePosition);

        return dot(PFree - QFree, normal);
    }

    static inline ViolationFunction getViolationFunc() {
        return std::bind(&ConstraintNormal::defaultViolationFunction, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3);
    }

    ConstraintNormal() {
    }

    ConstraintNormal(const ConstraintNormal &cn) {
        m_dirs = cn.m_dirs;
        m_functions = cn.m_functions;
    }

    ConstraintNormal(const type::Vec3 &N, ViolationFunction f = getViolationFunc()) {
        m_dirs.push_back(N.normalized());
        m_functions.push_back(f);
    }

    ConstraintNormal &add(const type::Vec3 &N, ViolationFunction f = getViolationFunc()) {
        m_dirs.push_back(N.normalized());
        m_functions.push_back(f);
        return *this;
    }

    /// returns ConstraintNormal vector's (m_dirs) size
    unsigned size() const {
        return m_dirs.size();
    }

    ConstraintNormal &addOrthogonalDirection(ViolationFunction f = getViolationFunc()) {
        if (m_dirs.size() == 0) {
            m_dirs.push_back(type::Vec3(1, 0, 0));
            m_functions.push_back(f);
            return *this;
        } else if (m_dirs.size() == 1) {
            // gram schmidt orthogonalization
            type::Vec3 gramm_schmidt = type::Vec3(0, 0, 1) - m_dirs[0] * dot(m_dirs[0], type::Vec3(0, 0, 1));

            // Change with othogonalization around Z if the initial direction is aligned with Y
            if (gramm_schmidt.norm() < std::numeric_limits<double>::epsilon())
                gramm_schmidt = type::Vec3(0, 1, 0) - m_dirs[0] * dot(m_dirs[0], type::Vec3(0, 1, 0));

            m_dirs.push_back(gramm_schmidt.normalized());
            m_functions.push_back(f);
            return *this;
        } else if (m_dirs.size() == 2) {
            m_dirs.push_back(cross(m_dirs[0], m_dirs[1]).normalized());
            m_functions.push_back(f);
            return *this;
        } else {
            std::cerr << "Cannot add orthogonal direction for more than 3 constraints " << std::endl;
        }

        return *this;
    }

    void computeViolations(unsigned cid, const typename FIRST::SPtr &first, const typename SECOND::SPtr &second,
                           linearalgebra::BaseVector *delta) const {
        for (unsigned i = 0; i < m_dirs.size(); i++) {
            double v = m_functions[i](first, second, m_dirs[i]);
            delta->set(cid + i, v);
        }
    }

    const type::Vec3 operator[](int i) const {
        return m_dirs[i];
    }

    ConstraintNormal &operator=(const ConstraintNormal &CN) {
        this->m_dirs = CN.m_dirs;
        this->m_functions = CN.m_functions;
        return *this;
    }

    const sofa::type::vector<type::Vec3> &getDirs() const {
        return m_dirs;
    }

    void scale(double s) {
        for (unsigned i = 0; i < m_dirs.size(); i++) m_dirs[i] *= s;
    }

    void setDir(unsigned i, const type::Vec3 &n) {
        m_dirs[i] = n.normalized();
    }

    ViolationFunction getViolationFunc(unsigned i) {
        return m_functions[i];
    }

protected:
    // pai of directions (vec3) and function to compute the violation of a par proximity
    sofa::type::vector<type::Vec3> m_dirs;
    sofa::type::vector<ViolationFunction> m_functions;
};

}  // namespace sofa::constraintgeometry
