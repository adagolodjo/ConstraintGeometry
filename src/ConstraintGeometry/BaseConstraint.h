#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <ConstraintGeometry/ConstraintDirection.h>
#include <ConstraintGeometry/ConstraintNormal.h>
#include <ConstraintGeometry/ConstraintResponse.h>
#include <ConstraintGeometry/InternalConstraint.h>
#include <ConstraintGeometry/config.h>
#include <sofa/core/behavior/BaseLagrangianConstraint.h>

namespace sofa::constraintgeometry {

class SOFA_CONSTRAINTGEOMETRY_API BaseConstraint : public sofa::core::behavior::BaseLagrangianConstraint {
public:
    virtual ~BaseConstraint() {
    }

    unsigned getCId() const {
        return this->d_constraintIndex.getValue();
    }

    unsigned getSize() const {
        return m_nbConstraints;
    }
    unsigned m_nbConstraints = 0;
};

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseLagrangianConstraint
 */
template <class FIRST = collisionalgorithm::BaseProximity, class SECOND = collisionalgorithm::BaseProximity>
class TBaseConstraint : public BaseConstraint {
public:
    SOFA_CLASS(BaseConstraint, sofa::core::behavior::BaseLagrangianConstraint);

    typedef collisionalgorithm::BaseProximity BaseProximity;

    Data<double> d_drawScale;
    Data<bool> d_draw;
    Data<collisionalgorithm::DetectionOutput<FIRST, SECOND> >
        d_input;  // THIS SHOULD BE REPLACED BY A PAIR OF CST PROXIMITY INPUT
    Data<std::vector<BaseInternalConstraint::SPtr> > d_container;

    TBaseConstraint()
        : d_drawScale(initData(&d_drawScale, 1.0, "draw_scale", "draw scale")),
          d_draw(initData(&d_draw, true, "draw", "draw ")),
          d_input(initData(&d_input, "input", "Link to detection output")),
          d_container(initData(&d_container, "internalConstraint", "Internal constraint vector")) {
    }

    virtual ConstraintNormal createConstraintNormal(const typename FIRST::SPtr& first,
                                                    const typename SECOND::SPtr& second) const = 0;

    virtual core::behavior::ConstraintResolution* createConstraintResolution(
        const BaseInternalConstraint* cst) const = 0;

    /*!
     * \brief processGeometricalData
     * Clears constraint container and recreates constraints
     */
    virtual void processGeometricalData() override {
        auto& container = *d_container.beginEdit();

        container.clear();  // clear previsou constraints

        auto& input = d_input.getValue();
        for (unsigned i = 0; i < input.size(); i++) {
            if (input[i].first == nullptr || input[i].second == nullptr) continue;

            auto normalCreator = [this](const typename FIRST::SPtr& first, const typename SECOND::SPtr& second) {
                return createConstraintNormal(first, second);
            };
            auto resolutionCreator = [this](const BaseInternalConstraint* cst) {
                return createConstraintResolution(cst);
            };

            auto constraint = InternalConstraint<FIRST, SECOND>::create(input[i].first, input[i].second, normalCreator,
                                                                        resolutionCreator);

            if (constraint->size()) container.push_back(constraint);
        }

        d_container.endEdit();
    }

    /*!
     * \brief buildConstraintMatrix loops through the constaint container and builds constraint matrix
     * \param cId
     * \param constraintId
     */
    virtual void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId,
                                       unsigned int& constraintId) override {
        setConstraintId(constraintId);
        m_nbConstraints = 0;
        for (unsigned i = 0; i < d_container.getValue().size(); i++) {
            d_container.getValue()[i]->buildConstraintMatrix(cId, constraintId);
            m_nbConstraints += d_container.getValue()[i]->size();
        }
    }

    /*!
     * \brief getConstraintViolation gets constraint violation for each constraint in constraint container
     * \param v
     */
    virtual void getConstraintViolation(const core::ConstraintParams* /*cParams*/, linearalgebra::BaseVector* v,
                                        unsigned /*cid*/) override {
        for (unsigned i = 0; i < d_container.getValue().size(); i++)
            d_container.getValue()[i]->getConstraintViolation(v);
    }

    /*!
     * \brief getConstraintResolution creates constraint resolution for each constaint in container
     * \param resTab
     * \param offset
     */
    virtual void getConstraintResolution(const core::ConstraintParams* /*cParams*/,
                                         std::vector<core::behavior::ConstraintResolution*>& resTab,
                                         unsigned int& offset) override {
        for (unsigned i = 0; i < d_container.getValue().size(); i++) {
            const unsigned size = d_container.getValue()[i]->size();
            if (offset + size > resTab.size()) {
                msg_error(this) << "Constraint resolution table is too small (" << resTab.size() << " entries, "
                                << (offset + size) << " required)";
                return;
            }

            resTab[offset] = d_container.getValue()[i]->createConstraintResolution();
            offset += size;
        }
    }

    void draw(const core::visual::VisualParams* vparams) override {
        if (!d_draw.getValue()) return;
        if (d_drawScale.getValue() > std::numeric_limits<double>::min()) {
            if (vparams->displayFlags().getShowInteractionForceFields()) {
                for (unsigned i = 0; i < d_container.getValue().size(); i++)
                    d_container.getValue()[i]->draw(vparams, d_drawScale.getValue());
            }

            if (vparams->displayFlags().getShowCollisionModels()) {
                glDisable(GL_LIGHTING);
                glColor4f(0, 1, 0, 1);

                glBegin(GL_LINES);
                for (unsigned i = 0; i < d_container.getValue().size(); i++) {
                    for (unsigned j = 0; j < d_container.getValue()[i]->getPairSize(); j++) {
                        glVertex3dv(d_container.getValue()[i]->getFirstPair(j)->getPosition().data());
                        glVertex3dv(d_container.getValue()[i]->getSecondPair(j)->getPosition().data());
                    }
                }
                glEnd();
            }
        }
    }

    /*!
     * \brief storeLambda, virtual method called for each container in storeLambda
     * \param cst : Internal Constraint Shared Pointer (typedef in Internal Constraint)
     * \param cParams
     * \param res
     * \param lambda
     */
    virtual void storeLambda(const BaseInternalConstraint::SPtr& cst, const core::ConstraintParams* cParams,
                             core::MultiVecDerivId res, const sofa::linearalgebra::BaseVector* lambda) {
        cst->storeLambda(cParams, res, lambda);
    }

    /*!
     * \brief storeLambda for each constraint in container
     * \param cParams
     * \param res
     * \param lambda
     */
    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res,
                             const sofa::linearalgebra::BaseVector* lambda) override {
        if (!cParams) return;

        for (unsigned i = 0; i < d_container.getValue().size(); i++)
            storeLambda(d_container.getValue()[i], cParams, res, lambda);
    }

protected:
};

}  // namespace sofa::constraintgeometry
