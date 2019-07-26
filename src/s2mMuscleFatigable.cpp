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
