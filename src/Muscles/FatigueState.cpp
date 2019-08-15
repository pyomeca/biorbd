#define BIORBD_API_EXPORTS
#include "Muscles/FatigueState.h"

#include <cmath>
#include "Utils/Error.h"

#if defined(_WIN32) || defined(_WIN64)
template < typename T > std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
#endif

biorbd::muscles::FatigueState::FatigueState(
        double active,
        double fatigued,
        double resting) :
    m_activeFibers(active),
    m_fatiguedFibers(fatigued),
    m_restingFibers(resting)
{
    setType();
}

biorbd::muscles::FatigueState::FatigueState(const std::shared_ptr<biorbd::muscles::FatigueState> m)
{
    m_activeFibers = m->m_activeFibers;
    m_fatiguedFibers = m->m_fatiguedFibers;
    m_restingFibers = m->m_restingFibers;
    m_type = m->m_type;
}

biorbd::muscles::FatigueState::~FatigueState()
{

}

void biorbd::muscles::FatigueState::setState(double active, double fatigued, double resting)
{
    // Sanity check for active fibers
    //
    // In order to get the quantity of active fibers to 0 or 1, it has to come from the input command.
    // The input command manage fiber recruitment from resting to active.
    // Hence any exceeding because of integration can be corrected by putting back exceeding active fibers quantity
    // into resting fibers quantity.
    //
    if (active < 0){
        resting += active;
        biorbd::utils::Error::warning(0, "Active Fibers Quantity can't be lower than 0, 0 is used then\n"
                                "Previous Active Fibers Quantity before set to 0:"+std::to_string(active));
        active = 0;
    }
    else if (active > 1){
        resting += active - 1;
        biorbd::utils::Error::warning(0, "Active Fibers Quantity can't be higher than 1, 1 is used then\n"
                                "Previous Active Fibers Quantity before set to 1: "+std::to_string(active));
        active = 1;
    }

    // Sanity check for fatigued fibers
    if (fatigued < 0){
        biorbd::utils::Error::error(0, "Fatigued Fibers Quantity can't be lower than 0");
    }
    else if (fatigued > 1){
        biorbd::utils::Error::error(0, "Fatigued Fibers Quantity can't be higher than 1");
    }

    // Sanity check for resting fibers
    //
    // In order to get the quantity of resting fibers to 0, it has to come from the input command.
    // The input command manage fiber recruitment from resting to active.
    // Hence any exceeding because of integration can be corrected by putting back exceeding resting fibers quantity
    // into active fibers quantity.
    //
    if (resting < 0){
        active += resting;
        biorbd::utils::Error::warning(0, "Resting Fibers Quantity can't be lower than 0, 0 is used then\n"
                                "Previous Resting Fibers Quantity before set to 0: "+std::to_string(resting));
        resting = 0;
    }
    else if (resting > 1){
        biorbd::utils::Error::error(0, "Resting Fibers Quantity can't be higher than 1");

    }

    if (fabs(active + fatigued + resting - 1.0) > 0.1){
        biorbd::utils::Error::error(false, "Sum of the fatigued states must be equal to 1");
    }

    m_activeFibers = active;
    m_fatiguedFibers = fatigued;
    m_restingFibers = resting;
}

double biorbd::muscles::FatigueState::activeFibers() const
{
    return m_activeFibers;
}

double biorbd::muscles::FatigueState::fatiguedFibers() const
{
    return m_fatiguedFibers;
}

double biorbd::muscles::FatigueState::restingFibers() const
{
    return m_restingFibers;
}

std::string biorbd::muscles::FatigueState::getType() const
{
    return m_type;
}

void biorbd::muscles::FatigueState::setType()
{
    m_type = "Simple";
}
