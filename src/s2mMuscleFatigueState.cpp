#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueState.h"

s2mMuscleFatigueState::s2mMuscleFatigueState(
        double active,
        double fatigued,
        double resting) :
    m_activeFibers(active),
    m_fatiguedFibers(fatigued),
    m_restingFibers(resting)
{
    setType();
}

s2mMuscleFatigueState::s2mMuscleFatigueState(
        const std::shared_ptr<s2mMuscle> m):
    m_activeFibers(0),
    m_fatiguedFibers(0),
    m_restingFibers(1)
{
    setType();
}

void s2mMuscleFatigueState::setState(double active, double fatigued, double resting)
{
    // Sanity check for active fibers
    if (active < 0){
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be lower than 0, 0 is used then");
        active = 0;
    }
    else if (active > 1){
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be higher than 1, 1 is used then");
        active = 1;
    }

    // Sanity check for fatigued fibers
    if (fatigued < 0){
        s2mError::s2mWarning(0, "Fatigued Fibers Quantity can't be lower than 0, 0 is used then");
        fatigued = 0;
    }
    else if (fatigued > 1){
        s2mError::s2mWarning(0, "Fatigued Fibers Quantity can't be higher than 1, 1 is used then");
        fatigued = 1;
    }

    // Sanity check for resting fibers
    if (resting < 0){
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be lower than 0, 0 is used then");
        resting = 0;
    }
    else if (resting > 1){
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be higher than 1, 1 is used then");
        resting = 1;
    }

    if (active + fatigued + resting != 1.0)
        s2mError::s2mAssert(false, "Sum of the fatigued states must be equal to 1");

    m_activeFibers = active;
    m_fatiguedFibers = fatigued;
    m_restingFibers = resting;
}

double s2mMuscleFatigueState::activeFibers() const
{
    return m_activeFibers;
}

double s2mMuscleFatigueState::fatiguedFibers() const
{
    return m_fatiguedFibers;
}

double s2mMuscleFatigueState::restingFibers() const
{
    return m_restingFibers;
}

std::string s2mMuscleFatigueState::getType() const
{
    return m_type;
}

void s2mMuscleFatigueState::setType()
{
    m_type = "Simple";
}
