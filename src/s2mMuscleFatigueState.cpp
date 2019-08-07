#define BIORBD_API_EXPORTS
#include "s2mMuscleFatigueState.h"

#include <cmath>
#include "s2mError.h"

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

s2mMuscleFatigueState::s2mMuscleFatigueState(const std::shared_ptr<s2mMuscleFatigueState> m)
{
    m_activeFibers = m->m_activeFibers;
    m_fatiguedFibers = m->m_fatiguedFibers;
    m_restingFibers = m->m_restingFibers;
    m_type = m->m_type;
}

s2mMuscleFatigueState::~s2mMuscleFatigueState()
{

}

void s2mMuscleFatigueState::setState(double active, double fatigued, double resting)
{
    // Sanity check for active fibers
    ///
    /// In order to get the quantity of active fibers to 0 or 1, it has to come from the input command.
    /// The input command manage fiber recruitment from resting to active.
    /// Hence any exceeding because of integration can be corrected by putting back exceeding active fibers quantity
    /// into resting fibers quantity.
    ///
    if (active < 0){
        resting += active;
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be lower than 0, 0 is used then\n"
                                "Previous Active Fibers Quantity before set to 0:"+std::to_string(active));
        active = 0;
    }
    else if (active > 1){
        resting += active - 1;
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be higher than 1, 1 is used then\n"
                                "Previous Active Fibers Quantity before set to 1: "+std::to_string(active));
        active = 1;
    }

    // Sanity check for fatigued fibers
    if (fatigued < 0){
        s2mError::s2mAssert(0, "Fatigued Fibers Quantity can't be lower than 0");
    }
    else if (fatigued > 1){
        s2mError::s2mAssert(0, "Fatigued Fibers Quantity can't be higher than 1");
    }

    // Sanity check for resting fibers
    ///
    /// In order to get the quantity of resting fibers to 0, it has to come from the input command.
    /// The input command manage fiber recruitment from resting to active.
    /// Hence any exceeding because of integration can be corrected by putting back exceeding resting fibers quantity
    /// into active fibers quantity.
    ///

    if (resting < 0){
        active += resting;
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be lower than 0, 0 is used then\n"
                                "Previous Resting Fibers Quantity before set to 0: "+std::to_string(resting));
        resting = 0;
    }
    else if (resting > 1){
        s2mError::s2mAssert(0, "Resting Fibers Quantity can't be higher than 1");

    }

    if (fabs(active + fatigued + resting - 1.0) > 0.1){
        s2mError::s2mAssert(false, "Sum of the fatigued states must be equal to 1");
    }

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
