#define BIORBD_API_EXPORTS
#include "Muscles/FatigueModel.h"

#include "Utils/Error.h"
#include "Muscles/Muscle.h"
#include "Muscles/FatigueDynamicStateXia.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

muscles::FatigueModel::FatigueModel(
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    if (dynamicFatigueType ==
            muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE) {
        m_fatigueState = std::make_shared<muscles::FatigueState>();
    } else if (dynamicFatigueType ==
               muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA) {
        m_fatigueState = std::make_shared<muscles::FatigueDynamicStateXia>();
    } else {
        utils::Error::raise("Wrong muscle fatigue type");
    }
}

muscles::FatigueModel::FatigueModel(
    const muscles::FatigueModel &other) :
    m_fatigueState(other.m_fatigueState)
{

}

muscles::FatigueModel::FatigueModel(
    const std::shared_ptr<muscles::FatigueModel> other) :
    m_fatigueState(other->m_fatigueState)
{

}

muscles::FatigueModel::~FatigueModel()
{

}

void muscles::FatigueModel::DeepCopy(const muscles::FatigueModel
        &other)
{
    *m_fatigueState = other.m_fatigueState->DeepCopy();
}

#ifndef BIORBD_USE_CASADI_MATH
void muscles::FatigueModel::setFatigueState(
    const utils::Scalar& active,
    const utils::Scalar& fatigued,
    const utils::Scalar& resting)
{
    m_fatigueState->setState(active, fatigued, resting);
}
#endif

muscles::FatigueState& muscles::FatigueModel::fatigueState()
{
    return *m_fatigueState;
}

const muscles::FatigueState&
muscles::FatigueModel::fatigueState() const
{
    return *m_fatigueState;
}

void muscles::FatigueModel::computeTimeDerivativeState(
    const muscles::StateDynamics &emg)
{
    if (std::dynamic_pointer_cast<muscles::FatigueDynamicState>
            (m_fatigueState)) {
        muscles::Muscle* muscle = dynamic_cast<muscles::Muscle*>(this);
        if (muscle) {
            std::static_pointer_cast<muscles::FatigueDynamicState>
            (m_fatigueState)->timeDerivativeState(emg,
                                                  muscle->characteristics());
        } else {
            utils::Error::raise("muscles::FatigueModel should be a muscles::Muscle");
        }
    } else {
        utils::Error::raise("Type cannot be fatigued");
    }
}

