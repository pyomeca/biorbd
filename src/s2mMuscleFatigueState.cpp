#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueState.h"

s2mMuscleFatigueState::s2mMuscleFatigueState(double active,
        double fatigued,
        double resting) :
    m_activeFibers(active),
    m_fatiguedFibers(fatigued),
    m_restingFibers(resting)
{

}

void s2mMuscleFatigueState::setState(double active, double fatigued, double resting)
{
    if (active + fatigued + resting != 1.0)
        s2mError::s2mAssert(false, "Sum of the fatigued states must be equal to 1");

    setActiveFibers(active);
    setFatiguedFibers(fatigued);
    setRestingFibers(resting);
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

void s2mMuscleFatigueState::setActiveFibers(double active)
{
    if (active < 0){
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be lower than 0, 0 is used then");
        m_activeFibers = 0;
    }
    else if (active > 1){
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be higher than 1, 1 is used then");
        m_activeFibers = 1;
    }
    else
        m_activeFibers = active;
}

void s2mMuscleFatigueState::setFatiguedFibers(double fatigued)
{
    if (fatigued < 0){
        s2mError::s2mWarning(0, "Fatigued Fibers Quantity can't be lower than 0, 0 is used then");
        m_fatiguedFibers = 0;
    }
    else if (fatigued > 1){
        s2mError::s2mWarning(0, "Fatigued Fibers Quantity can't be higher than 1, 1 is used then");
        m_fatiguedFibers = 1;
    }
    else
        m_fatiguedFibers = fatigued;
}

void s2mMuscleFatigueState::setRestingFibers(double resting)
{
    // Setting m_restingFibers
    if (resting < 0){
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be lower than 0, 0 is used then");
        m_restingFibers = 0;
    }
    else if (resting > 1){
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be higher than 1, 1 is used then");
        m_restingFibers = 1;
    }
    else
        m_restingFibers = resting;
}
