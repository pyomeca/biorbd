#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigable.h"

s2mMuscleFatigable::s2mMuscleFatigable(const s2mString &dynamicFatigueType)
{
    if (!dynamicFatigueType.tolower().compare("simple"))
        m_fatigueState = std::make_shared<s2mMuscleFatigueState>();
    else if (!dynamicFatigueType.tolower().compare("xia"))
        m_fatigueState = std::make_shared<s2mMuscleFatigueDynamicStateXia>();
    else
        s2mError::s2mAssert(false, "Wrong muscle fatigue type");
}

s2mMuscleFatigable::s2mMuscleFatigable(const s2mMuscle &m)
{
    try {
        const s2mMuscleFatigable& m_tp(dynamic_cast<const s2mMuscleFatigable&>(m));
        this->m_fatigueState = m_tp.m_fatigueState;
    } catch (const std::bad_cast&) {
        s2mError::s2mAssert(false, "This muscle is not fatigable");
    }
}

s2mMuscleFatigable::s2mMuscleFatigable(const std::shared_ptr<s2mMuscle> m)
{
    const std::shared_ptr<s2mMuscleFatigable> m_tp(std::dynamic_pointer_cast<s2mMuscleFatigable>(m));
    if (!m_tp)
        s2mError::s2mAssert(false, "This muscle is not fatigable");
    this->m_fatigueState = m_tp->m_fatigueState;
}

s2mMuscleFatigable::~s2mMuscleFatigable()
{

}

std::shared_ptr<s2mMuscleFatigueState> s2mMuscleFatigable::fatigueState()
{
    return m_fatigueState;
}

void s2mMuscleFatigable::fatigueState(double active, double fatigued, double resting)
{
    m_fatigueState->setState(active, fatigued, resting);
}


void s2mMuscleFatigable::computeTimeDerivativeState(const s2mMuscleStateActual &EMG)
{
    if (std::dynamic_pointer_cast<s2mMuscleFatigueDynamicState>(m_fatigueState)) {
        s2mMuscle* muscle = dynamic_cast<s2mMuscle*>(this);
        if (muscle)
            std::static_pointer_cast<s2mMuscleFatigueDynamicState>(m_fatigueState)->timeDerivativeState(EMG, muscle->caract());
        else
            s2mError::s2mAssert(false, "s2mMuscleFatigable should be a s2mMuscle");
   } else {
       s2mError::s2mAssert(false, "Type cannot be fatigued");
    }
}
