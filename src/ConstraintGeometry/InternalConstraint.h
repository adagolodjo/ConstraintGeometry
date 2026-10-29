#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <ConstraintGeometry/ConstraintNormal.h>
#include <ConstraintGeometry/config.h>
#include <sofa/core/behavior/BaseLagrangianConstraint.h>
#include <sofa/type/vector.h>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API BaseInternalConstraint {
public:
    typedef std::shared_ptr<BaseInternalConstraint> SPtr;

    virtual unsigned size() const = 0;

    virtual void scale(double s) = 0;

    virtual unsigned id() const = 0;

    virtual void toString(std::ostream& out) const = 0;

    virtual void buildConstraintMatrix(core::MultiMatrixDerivId, unsigned int&) const = 0;

    virtual void storeLambda(const core::ConstraintParams*, core::MultiVecDerivId,
                             const sofa::linearalgebra::BaseVector*) const = 0;

    virtual void getConstraintViolation(linearalgebra::BaseVector*) const = 0;

    virtual core::behavior::ConstraintResolution* createConstraintResolution() const = 0;

    virtual unsigned getPairSize() const = 0;

    virtual collisionalgorithm::BaseBaseProximity::SPtr getFirstPair(unsigned i) const = 0;

    virtual collisionalgorithm::BaseBaseProximity::SPtr getSecondPair(unsigned i) const = 0;

    virtual void draw(const core::visual::VisualParams*, double) const = 0;

    virtual const std::vector<ConstraintNormal>& getConstraintNormal() const = 0;

    friend inline std::istream& operator>>(std::istream& in, std::vector<BaseInternalConstraint::SPtr>&) {
        return in;
    }

    friend inline std::ostream& operator<<(std::ostream& out, const std::vector<BaseInternalConstraint::SPtr>& v) {
        for (unsigned i = 0; i < v.size(); i++) {
            if (v.size() > 1) out << "[";
            v[i]->toString(out);
            if (v.size() > 1) out << "]" << std::endl;
        }
        return out;
    }
};

template <class FIRST, class SECOND>
class InternalConstraint : public BaseInternalConstraint {
public:
    typedef std::shared_ptr<InternalConstraint<FIRST, SECOND>> SPtr;

    //    typedef collisionalgorithm::BaseProximity FIRST;
    //    typedef collisionalgorithm::BaseProximity SECOND;

    typedef collisionalgorithm::BaseProximity BaseProximity;
    typedef std::function<core::behavior::ConstraintResolution*(const BaseInternalConstraint*)> ResolutionCreator;
    typedef std::function<void(const InternalConstraint*, linearalgebra::BaseVector*)> ViolationFunc;
    typedef std::function<ConstraintNormal(const typename FIRST::SPtr&, const typename SECOND::SPtr&)> NormalsFunc;

    static void defaultViolationFunc(const InternalConstraint* ic, linearalgebra::BaseVector* v) {
        // the ConstraintNormal will compute the all violation (i.e. 1 to 3 depending on the size of the
        // ConstraintNormal)
        unsigned int lcid = ic->m_cid;
        for (unsigned i = 0; i < ic->m_vecNormals.size(); i++) {
            ic->m_vecNormals[i].computeViolations(lcid, ic->m_pairs[i].first, ic->m_pairs[i].second, v);

            /*++ lcid;*/ lcid += ic->m_vecNormals[i].size();
        }
    }

    // create function should be used
    /*!
     * \brief InternalConstraint Constructor
     * \param p1 : 1st proximity
     * \param p2 : second proximity
     * \param normals : ConstraintNormals
     * \param creator : resolutionCreator (factory)
     */
    InternalConstraint(const typename FIRST::SPtr& first, const typename SECOND::SPtr& second, NormalsFunc nf,
                       ResolutionCreator creator, ViolationFunc f = &defaultViolationFunc)
        : m_creator(creator), m_cid(0), m_cSetId(0), m_cDirId(0), m_violationFunc(f), m_normalFunc(nf) {
        m_pairs.push_back(std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>(first, second));
        m_vecNormals.push_back(m_normalFunc(first, second));
    }

    InternalConstraint(const std::vector<std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>>& pairs,
                       NormalsFunc nf, ResolutionCreator creator, ViolationFunc f = &defaultViolationFunc)
        : m_creator(creator), m_cid(0), m_cSetId(0), m_cDirId(0), m_violationFunc(f), m_normalFunc(nf) {
        for (unsigned i = 0; i < pairs.size(); i++) {
            m_pairs.push_back(
                std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>(pairs[i].first, pairs[i].second));
            m_vecNormals.push_back(m_normalFunc(pairs[i].first, pairs[i].second));
        }
    }

    static inline SPtr create(const typename FIRST::SPtr& first, const typename SECOND::SPtr& second, NormalsFunc nf,
                              ResolutionCreator creator, ViolationFunc f = &defaultViolationFunc) {
        return SPtr(new InternalConstraint(first, second, nf, creator, f));
    }

    static inline SPtr create(
        const std::vector<std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>>& pairs, NormalsFunc nf,
        ResolutionCreator creator, ViolationFunc f = &defaultViolationFunc) {
        return SPtr(new InternalConstraint(pairs, nf, creator, f));
    }

    /*!
     * \brief buildConstraintMatrix
     * Builds proximities p1 & p2's Jacobian Constraints
     * \param cId
     * \param constraintId
     */
    void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int& constraintId) const override {
        m_cid = constraintId;

        for (unsigned i = 0; i < m_vecNormals.size(); i++) {
            m_pairs[i].first->buildJacobianConstraint(cId, m_vecNormals[i].getDirs(), 1.0, constraintId);
            m_pairs[i].second->buildJacobianConstraint(cId, m_vecNormals[i].getDirs(), -1.0, constraintId);

            constraintId += m_vecNormals[i].size();
        }
    }

    void getConstraintViolation(linearalgebra::BaseVector* v) const override {
        m_violationFunc(this, v);
    }

    /*!
     * \brief createConstraintResolution, creates ConstraintResolution using creator's factory
     * \return ConstraintResolution*
     */
    core::behavior::ConstraintResolution* createConstraintResolution() const override {
        return m_creator(this);
    }

    /*!
     * \brief storeLambda, calls storeLambda on p1 and p2 proximities' storeLambda
     * \param cParams
     * \param res
     * \param lambda
     */
    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res,
                     const sofa::linearalgebra::BaseVector* lambda) const override {
        unsigned int lcid = m_cid;
        for (unsigned k = 0; k < m_vecNormals.size(); k++) {
            for (unsigned i = 0; i < m_vecNormals[k].size(); i++) {
                m_pairs[k].first->storeLambda(cParams, res, /*m_cid*/ lcid, i, lambda);
                m_pairs[k].second->storeLambda(cParams, res, /*m_cid*/ lcid, i, lambda);
            }
            lcid += m_vecNormals[k].size();
        }
    }

    /*!
     * \brief getLambda
     * \param lambda : const BaseVector*
     * \param id : unsigned
     * \return lambda element @ cid+id
     */
    double getLambda(const sofa::linearalgebra::BaseVector* lambda, unsigned id) {
        return lambda->element(m_cid + id);
    }

    void draw(const core::visual::VisualParams* vparams, double scale) const override {
        for (unsigned i = 0; i < m_vecNormals.size(); i++) {
            draw(vparams, scale, i);
        }
    }

    void draw(const core::visual::VisualParams* vparams, double scale, unsigned i) const {
        if (m_vecNormals[i].size() > 0) {
            vparams->drawTool()->drawArrow(m_pairs[i].first->getPosition(),
                                           m_pairs[i].first->getPosition() + m_vecNormals[i].getDirs()[0] * scale,
                                           scale * 0.1, type::RGBAColor(1, 0, 0, 1));
            vparams->drawTool()->drawArrow(m_pairs[i].second->getPosition(),
                                           m_pairs[i].second->getPosition() - m_vecNormals[i].getDirs()[0] * scale,
                                           scale * 0.1, type::RGBAColor(1, 0, 0, 1));
        }

        if (m_vecNormals[i].size() > 1) {
            vparams->drawTool()->drawArrow(m_pairs[i].first->getPosition(),
                                           m_pairs[i].first->getPosition() + m_vecNormals[i].getDirs()[1] * scale,
                                           scale * 0.1, type::RGBAColor(0, 1, 0, 1));
            vparams->drawTool()->drawArrow(m_pairs[i].second->getPosition(),
                                           m_pairs[i].second->getPosition() - m_vecNormals[i].getDirs()[1] * scale,
                                           scale * 0.1, type::RGBAColor(0, 1, 0, 1));
        }

        if (m_vecNormals[i].size() > 2) {
            vparams->drawTool()->drawArrow(m_pairs[i].first->getPosition(),
                                           m_pairs[i].first->getPosition() + m_vecNormals[i].getDirs()[2] * scale,
                                           scale * 0.1, type::RGBAColor(0, 0, 1, 1));
            vparams->drawTool()->drawArrow(m_pairs[i].second->getPosition(),
                                           m_pairs[i].second->getPosition() - m_vecNormals[i].getDirs()[2] * scale,
                                           scale * 0.1, type::RGBAColor(0, 0, 1, 1));
        }
    }

    const std::vector<ConstraintNormal>& getConstraintNormal() const override {
        return m_vecNormals;
    }

    unsigned getPairSize() const override {
        return m_pairs.size();
    }

    collisionalgorithm::BaseBaseProximity::SPtr getFirstPair(unsigned i) const override {
        if (i >= m_pairs.size()) return nullptr;
        return m_pairs[i].first;
    }

    collisionalgorithm::BaseBaseProximity::SPtr getSecondPair(unsigned i) const override {
        if (i >= m_pairs.size()) return nullptr;
        return m_pairs[i].second;
    }

    const std::vector<std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>>& getPairs() const {
        return m_pairs;
    }

    unsigned size() const override {
        unsigned lcid = 0;
        for (unsigned i = 0; i < m_vecNormals.size(); i++) {
            lcid += m_vecNormals[i].size();
        }
        return lcid;
    }

    unsigned id() const override {
        return m_cid;
    }

    void toString(std::ostream& out) const override {
        out << "(" << m_cid << ") : ";
        for (unsigned i = 0; i < m_pairs.size(); i++) {
            if (m_pairs.size() > 1) out << "<";
            out << sofa::helper::NameDecoder::decodeFullName(m_pairs[i].first->getTypeInfo()) << "    ---   "
                << sofa::helper::NameDecoder::decodeFullName(m_pairs[i].second->getTypeInfo()) << std::endl;
            out << "  - " << m_pairs[i].first->getPosition() << std::endl;
            out << "  - " << m_pairs[i].second->getPosition() << std::endl;

            if (m_pairs.size() > 1) out << ">" << std::endl;
        }
    }

    unsigned constraintSetId() const {
        return m_cSetId;
    }

    unsigned constraintDirId() const {
        return m_cDirId;
    }

    void scale(double s) override {
        for (unsigned i = 0; i < m_vecNormals.size(); i++) m_vecNormals[i].scale(s);
    }

    void operator=(const InternalConstraint& c) {
        if (this == &c) return;

        m_pairs.clear();
        for (unsigned i = 0; i < c.m_pairs.size(); i++) m_pairs.push_back(c.m_pairs[i]);

        m_vecNormals = c.m_vecNormals;
        m_creator = c.m_creator;
        m_cid = c.m_cid;

        m_cSetId = c.m_cSetId;
        m_cDirId = c.m_cDirId;
        m_violationFunc = c.m_violationFunc;
        m_normalFunc = c.m_normalFunc;
    }

    friend inline std::istream& operator>>(
        std::istream& in, std::vector<typename sofa::constraintgeometry::InternalConstraint<FIRST, SECOND>::SPtr>&) {
        return in;
    }

    friend inline std::ostream& operator<<(
        std::ostream& out,
        const std::vector<typename sofa::constraintgeometry::InternalConstraint<FIRST, SECOND>::SPtr>& v) {
        for (unsigned i = 0; i < v.size(); i++) {
            if (v.size() > 1) out << "[";
            v[i]->toString(out);
            if (v.size() > 1) out << "]" << std::endl;
        }
        return out;
    }

    NormalsFunc getNormalFunc() {
        return m_normalFunc;
    }

    NormalsFunc getViolationFunc() {
        return m_violationFunc;
    }

    const std::vector<std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>>& getPairs() {
        return m_pairs;
    }

protected:
    std::vector<std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>> m_pairs;

    std::vector<ConstraintNormal> m_vecNormals;
    ResolutionCreator m_creator;
    mutable unsigned m_cid;

    mutable unsigned m_cSetId;
    mutable unsigned m_cDirId;
    ViolationFunc m_violationFunc;
    NormalsFunc m_normalFunc;
};

}  // namespace sofa::constraintgeometry
