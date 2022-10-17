#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/FatigueModel.h"

#include "Utils/Error.h"
#include "InternalForces/Muscles/Muscle.h"
#include "InternalForces/Muscles/FatigueDynamicStateXia.h"

using namespace BIORBD_NAMESPACE;

internalforce::muscles::FatigueModel::FatigueModel(
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    if (dynamicFatigueType ==
            internalforce::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE) {
        m_fatigueState = std::make_shared<internalforce::muscles::FatigueState>();
    } else if (dynamicFatigueType ==
               internalforce::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA) {
        m_fatigueState = std::make_shared<internalforce::muscles::FatigueDynamicStateXia>();
    } else {
        utils::Error::raise("Wrong muscle fatigue type");
    }
}

internalforce::muscles::FatigueModel::FatigueModel(
    const internalforce::muscles::FatigueModel &other) :
    m_fatigueState(other.m_fatigueState)
{

}

internalforce::muscles::FatigueModel::FatigueModel(
    const std::shared_ptr<internalforce::muscles::FatigueModel> other) :
    m_fatigueState(other->m_fatigueState)
{

}

internalforce::muscles::FatigueModel::~FatigueModel()
{

}

void internalforce::muscles::FatigueModel::DeepCopy(const internalforce::muscles::FatigueModel
        &other)
{
    *m_fatigueState = other.m_fatigueState->DeepCopy();
}

#ifndef BIORBD_USE_CASADI_MATH
void internalforce::muscles::FatigueModel::setFatigueState(
    const utils::Scalar& active,
    const utils::Scalar& fatigued,
    const utils::Scalar& resting)
{
    m_fatigueState->setState(active, fatigued, resting);
}
#endif

internalforce::muscles::FatigueState& internalforce::muscles::FatigueModel::fatigueState()
{
    return *m_fatigueState;
}

const internalforce::muscles::FatigueState&
internalforce::muscles::FatigueModel::fatigueState() const
{
    return *m_fatigueState;
}

void internalforce::muscles::FatigueModel::computeTimeDerivativeState(
    const internalforce::muscles::StateDynamics &emg)
{
    if (std::dynamic_pointer_cast<internalforce::muscles::FatigueDynamicState>
            (m_fatigueState)) {
        internalforce::muscles::Muscle* muscle = dynamic_cast<internalforce::muscles::Muscle*>(this);
        if (muscle) {
            std::static_pointer_cast<internalforce::muscles::FatigueDynamicState>
            (m_fatigueState)->timeDerivativeState(emg,
                                                  muscle->characteristics());
        } else {
            utils::Error::raise("internalforce::muscles::FatigueModel should be a internalforce::muscles::Muscle");
        }
    } else {
        utils::Error::raise("Type cannot be fatigued");
    }
}

