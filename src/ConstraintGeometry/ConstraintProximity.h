#pragma once

#include <ConstraintGeometry/config.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <memory>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API ConstraintProximity : public virtual collisionalgorithm::BaseBaseProximity {
public:

    typedef std::shared_ptr<ConstraintProximity> SPtr;

    virtual type::Vec3 getNormal() const = 0;

    virtual sofa::type::Vec3 getPosition(core::VecCoordId = core::vec_id::write_access::position) const override { return  sofa::type::Vec3{}; }

    virtual sofa::type::Vec3 getVelocity(core::VecDerivId = core::vec_id::write_access::velocity) const override { return  sofa::type::Vec3{}; }

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId , const sofa::type::vector<sofa::type::Vec3> & , double , Index ) const override {}

    virtual void storeLambda(const core::ConstraintParams* , core::MultiVecDerivId , Index , Index , const sofa::linearalgebra::BaseVector* ) const override {}

};

template<class PROXIMITY>
class TConstraintProximity : public ConstraintProximity {
public:
    typedef std::shared_ptr<TConstraintProximity<PROXIMITY> > SPtr;
    typedef std::function<type::Vec3(const typename PROXIMITY::SPtr & p)> FUNC_GET_NORMAL;

    TConstraintProximity(const typename PROXIMITY::SPtr & p,FUNC_GET_NORMAL n)
    : m_prox(p)
    , m_normalFunc(n) {}

    sofa::type::Vec3 getPosition(core::VecCoordId v = core::vec_id::write_access::position) const override {
        return m_prox->getPosition(v);
    }

    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::vec_id::write_access::velocity) const override {
        return m_prox->getVelocity(v);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vec3> & dir, double fact, Index constraintId) const override {
        m_prox->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_prox->storeLambda(cParams,resId,cid_global,cid_local,lambda);
    }

    type::Vec3 getNormal() const override {
        return m_normalFunc(m_prox);
    }

    static typename TConstraintProximity<PROXIMITY>::SPtr create(const typename PROXIMITY::SPtr & p,FUNC_GET_NORMAL n) {
        return std::make_shared<TConstraintProximity<PROXIMITY>>(p, n);
    }

    const typename PROXIMITY::SPtr & getProx() const { return m_prox; }

    FUNC_GET_NORMAL getNormalFunc() const { return m_normalFunc; }

protected:
    typename PROXIMITY::SPtr m_prox;
    FUNC_GET_NORMAL m_normalFunc;
};

template<class PROXIMITY>
class BaseProximityNormal : public PROXIMITY, public constraintgeometry::ConstraintProximity {
public:
    typedef std::shared_ptr<BaseProximityNormal> SPtr;

    template<class... ARGS>
    BaseProximityNormal(ARGS... args)
    : PROXIMITY(args...) {}

    sofa::type::Vec3 getPosition(core::VecCoordId v = core::vec_id::write_access::position) const override {
        return PROXIMITY::getPosition(v);
    }

    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::vec_id::write_access::velocity) const override {
        return PROXIMITY::getVelocity(v);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId id, const sofa::type::vector<sofa::type::Vec3> & N, double f, Index i) const override {
        PROXIMITY::buildJacobianConstraint(id,N,f,i);
    }

    void storeLambda(const core::ConstraintParams* cp, core::MultiVecDerivId id, Index i, Index j, const sofa::linearalgebra::BaseVector* l) const override {
        PROXIMITY::storeLambda(cp,id,i,j,l);
    }

    virtual sofa::type::Vec3 getNormal() const = 0;
};



class DefaultConstraintProximity : public ConstraintProximity {
public:

    typedef std::shared_ptr<DefaultConstraintProximity> SPtr;

    DefaultConstraintProximity(const collisionalgorithm::BaseProximity::SPtr & p)
    : m_prox(p) {}

    sofa::type::Vec3 getPosition(core::VecCoordId v = core::vec_id::write_access::position) const override {
        return m_prox->getPosition(v);
    }

    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::vec_id::write_access::velocity) const override {
        return m_prox->getVelocity(v);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vec3> & dir, double fact, Index constraintId) const override {
        m_prox->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_prox->storeLambda(cParams,resId,cid_global,cid_local,lambda);
    }

    type::Vec3 getNormal() const override {
        return type::Vec3();
    }

    static ConstraintProximity::SPtr create(const collisionalgorithm::BaseProximity::SPtr & p) {
        return std::make_shared<DefaultConstraintProximity>(p);
    }

protected:
    collisionalgorithm::BaseProximity::SPtr m_prox;
};



class MultiConstraintProximity : public ConstraintProximity {
public:
    typedef std::shared_ptr<MultiConstraintProximity> SPtr;

    MultiConstraintProximity(const std::vector<ConstraintProximity::SPtr> & p)
    : m_proximities(p) {}

    sofa::type::Vec3 getPosition(core::VecCoordId v = core::vec_id::write_access::position) const override {
        sofa::type::Vec3 P(0,0,0);
        for (unsigned i=0; i<m_proximities.size(); i++) P+=m_proximities[i]->getPosition(v);
        return P*1.0/m_proximities.size();
    }

    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::vec_id::write_access::velocity) const override {
        sofa::type::Vec3 P(0,0,0);
        for (unsigned i=0; i<m_proximities.size(); i++) P+=m_proximities[i]->getVelocity(v);
        return P*1.0/m_proximities.size();
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vec3> & dir, double fact, Index constraintId) const override {
        fact *= 1.0/m_proximities.size();
        for (unsigned i=0; i<m_proximities.size(); i++) {
            m_proximities[i]->buildJacobianConstraint(cId,dir,fact,constraintId);
        }
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        for (unsigned i=0; i<m_proximities.size(); i++) {
            m_proximities[i]->storeLambda(cParams,resId,cid_global,cid_local,lambda);
        }
    }

    type::Vec3 getNormal() const override {
        sofa::type::Vec3 N(0,0,0);
        for (unsigned i=0; i<m_proximities.size(); i++) N+=m_proximities[i]->getNormal();
        return N*1.0/m_proximities.size();
    }

    static ConstraintProximity::SPtr create(const std::vector<ConstraintProximity::SPtr> & p) {
        return std::make_shared<MultiConstraintProximity>(p);
    }

protected:
    std::vector<ConstraintProximity::SPtr> m_proximities;
};

}
