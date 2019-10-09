#define BIORBD_API_EXPORTS
#include "Muscles/Fatigable.h"

#include "Utils/Error.h"
#include "Muscles/Muscle.h"
#include "Muscles/FatigueDynamicStateXia.h"

biorbd::muscles::Fatigable::Fatigable(
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    if (dynamicFatigueType == biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
        m_fatigueState = std::make_shared<biorbd::muscles::FatigueState>();
    else if (dynamicFatigueType == biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA)
        m_fatigueState = std::make_shared<biorbd::muscles::FatigueDynamicStateXia>();
    else
        biorbd::utils::Error::raise("Wrong muscle fatigue type");
}

biorbd::muscles::Fatigable::Fatigable(
        const biorbd::muscles::Fatigable &m) :
    m_fatigueState(m.m_fatigueState)
{

}

biorbd::muscles::Fatigable::Fatigable(
        const std::shared_ptr<biorbd::muscles::Fatigable> m) :
    m_fatigueState(m->m_fatigueState)
{

}

biorbd::muscles::Fatigable::~Fatigable()
{

}

void biorbd::muscles::Fatigable::DeepCopy(const biorbd::muscles::Fatigable &other)
{
    *m_fatigueState = other.m_fatigueState->DeepCopy();
}

biorbd::muscles::FatigueState& biorbd::muscles::Fatigable::fatigueState()
{
    return *m_fatigueState;
}

const biorbd::muscles::FatigueState& biorbd::muscles::Fatigable::fatigueState() const
{
    return *m_fatigueState;
}

void biorbd::muscles::Fatigable::fatigueState(double active, double fatigued, double resting)
{
    m_fatigueState->setState(active, fatigued, resting);
}


void biorbd::muscles::Fatigable::computeTimeDerivativeState(const biorbd::muscles::StateDynamics &emg)
{
    if (std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicState>(m_fatigueState)) {
        biorbd::muscles::Muscle* muscle = dynamic_cast<biorbd::muscles::Muscle*>(this);
        if (muscle)
            std::static_pointer_cast<biorbd::muscles::FatigueDynamicState>(m_fatigueState)->timeDerivativeState(emg, muscle->caract());
        else
            biorbd::utils::Error::raise("biorbd::muscles::Fatigable should be a biorbd::muscles::Muscle");
   } else {
       biorbd::utils::Error::raise("Type cannot be fatigued");
    }
}

