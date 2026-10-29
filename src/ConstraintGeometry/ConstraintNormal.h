#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <ConstraintGeometry/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/type/vector.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

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
        if (first == nullptr || second == nullptr) return 0.0;

        const type::Vec3 &PFree = first->getPosition(core::vec_id::write_access::freePosition);
        const type::Vec3 &QFree = second->getPosition(core::vec_id::write_access::freePosition);

        return dot(PFree - QFree, normal);
    }

    static inline ViolationFunction getViolationFunc() {
        return [](const typename FIRST::SPtr &first, const typename SECOND::SPtr &second, const type::Vec3 &normal) {
            return defaultViolationFunction(first, second, normal);
        };
    }

    ConstraintNormal() {
    }

    ConstraintNormal(const ConstraintNormal &cn) {
        m_dirs = cn.m_dirs;
        m_functions = cn.m_functions;
    }

    ConstraintNormal(const type::Vec3 &N, ViolationFunction f = getViolationFunc()) {
        add(N, std::move(f));
    }

    ConstraintNormal &add(const type::Vec3 &N, ViolationFunction f = getViolationFunc()) {
        if (!isValidDirection(N)) return *this;

        m_dirs.push_back(N.normalized());
        m_functions.push_back(std::move(f));
        return *this;
    }

    /// returns ConstraintNormal vector's (m_dirs) size
    unsigned size() const {
        return m_dirs.size();
    }

    /// The two unit directions orthogonal to `axis` (and to each other): the transverse directions of a sliding constraint.
    /// Empty if `axis` is null.
    static ConstraintNormal transverse(const type::Vec3 &axis) {
        ConstraintNormal axisOnly(axis);
        ConstraintNormal result;
        if (axisOnly.size() == 0) return result;
        axisOnly.addOrthogonalDirection();
        axisOnly.addOrthogonalDirection();
        result.add(axisOnly[1]);
        result.add(axisOnly[2]);
        return result;
    }

    ConstraintNormal &addOrthogonalDirection(ViolationFunction f = getViolationFunc()) {
        if (m_dirs.size() == 0) {
            return add(type::Vec3(1, 0, 0), std::move(f));
        } else if (m_dirs.size() == 1) {
            // gram schmidt orthogonalization
            type::Vec3 gramm_schmidt = type::Vec3(0, 0, 1) - m_dirs[0] * dot(m_dirs[0], type::Vec3(0, 0, 1));

            // Change with othogonalization around Z if the initial direction is aligned with Y
            if (gramm_schmidt.norm() < std::numeric_limits<double>::epsilon())
                gramm_schmidt = type::Vec3(0, 1, 0) - m_dirs[0] * dot(m_dirs[0], type::Vec3(0, 1, 0));

            return add(gramm_schmidt, std::move(f));
        } else if (m_dirs.size() == 2) {
            return add(cross(m_dirs[0], m_dirs[1]), std::move(f));
        } else {
            std::cerr << "Cannot add orthogonal direction for more than 3 constraints " << std::endl;
        }

        return *this;
    }

    void computeViolations(unsigned cid, const typename FIRST::SPtr &first, const typename SECOND::SPtr &second,
                           linearalgebra::BaseVector *delta) const {
        const unsigned count = std::min<unsigned>(m_dirs.size(), m_functions.size());
        for (unsigned i = 0; i < count; i++) {
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
        if (i < m_dirs.size() && isValidDirection(n)) m_dirs[i] = n.normalized();
    }

    ViolationFunction getViolationFunc(unsigned i) {
        if (i < m_functions.size()) return m_functions[i];
        return getViolationFunc();
    }

    /// Replaces the violation function of direction i (ignored if i is out of range).
    void setViolationFunc(unsigned i, ViolationFunction f) {
        if (i < m_functions.size()) m_functions[i] = std::move(f);
    }

    /// Shifts the violation of direction i by -offset (a gap larger than offset is not a violation): used to give
    /// a contact a thickness (e.g. the radius of a rod). Ignored if i is out of range or offset is 0.
    void addViolationOffset(unsigned i, double offset) {
        if (i >= m_functions.size() || offset == 0.0) return;
        ViolationFunction previous = m_functions[i];
        m_functions[i] = [previous, offset](const typename FIRST::SPtr &first, const typename SECOND::SPtr &second,
                                            const type::Vec3 &normal) { return previous(first, second, normal) - offset; };
    }

protected:
    static bool isValidDirection(const type::Vec3 &n) {
        return n.norm() > std::numeric_limits<double>::epsilon();
    }

    // pai of directions (vec3) and function to compute the violation of a par proximity
    sofa::type::vector<type::Vec3> m_dirs;
    sofa::type::vector<ViolationFunction> m_functions;
};

}  // namespace sofa::constraintgeometry
