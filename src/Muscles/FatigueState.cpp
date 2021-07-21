#define BIORBD_API_EXPORTS
#include "Muscles/FatigueState.h"

#include <cmath>
#include "Utils/Error.h"

#if defined(_WIN32) || defined(_WIN64)
#include <sstream>
namespace std
{
template < typename T > std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
}
#endif

using namespace BIORBD_NAMESPACE;

muscles::FatigueState::FatigueState(
    const utils::Scalar& active,
    const utils::Scalar& fatigued,
    const utils::Scalar& resting) :
    m_activeFibers(std::make_shared<utils::Scalar>(active)),
    m_fatiguedFibers(std::make_shared<utils::Scalar>(fatigued)),
    m_restingFibers(std::make_shared<utils::Scalar>(resting)),
    m_type(std::make_shared<muscles::STATE_FATIGUE_TYPE>())
{
    setType();
}

muscles::FatigueState::FatigueState(const muscles::FatigueState
        &other) :
    m_activeFibers(other.m_activeFibers),
    m_fatiguedFibers(other.m_fatiguedFibers),
    m_restingFibers(other.m_restingFibers),
    m_type(other.m_type)
{

}

muscles::FatigueState::FatigueState(const
        std::shared_ptr<muscles::FatigueState> other) :
    m_activeFibers(other->m_activeFibers),
    m_fatiguedFibers(other->m_fatiguedFibers),
    m_restingFibers(other->m_restingFibers),
    m_type(other->m_type)
{

}

muscles::FatigueState::~FatigueState()
{

}

muscles::FatigueState muscles::FatigueState::DeepCopy() const
{
    muscles::FatigueState copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::FatigueState::DeepCopy(const muscles::FatigueState
        &other)
{
    *m_activeFibers = *other.m_activeFibers;
    *m_fatiguedFibers = *other.m_fatiguedFibers;
    *m_restingFibers = *other.m_restingFibers;
    *m_type = *other.m_type;
}

#ifndef BIORBD_USE_CASADI_MATH
void muscles::FatigueState::setState(
    utils::Scalar active,
    utils::Scalar fatigued,
    utils::Scalar resting,
    bool turnOffWarnings)
{
    // Sanity check for active fibers
    //
    // In order to get the quantity of active fibers to 0 or 1, it has to come from the input command.
    // The input command manage fiber recruitment from resting to active.
    // Hence any exceeding because of integration can be corrected by putting back exceeding active fibers quantity
    // into resting fibers quantity.
    //
    if (active < 0) {
        if (!turnOffWarnings) {
            utils::Error::warning(0,
                                          "Active Fibers Quantity can't be lower than 0, 0 is used then\n"
                                          "Previous Active Fibers Quantity before set to 0:"
                                          + std::to_string(active));
        }
        resting += active;
        active = 0;
    } else if (active > 1) {
        if (!turnOffWarnings) {
            utils::Error::warning(0,
                                          "Active Fibers Quantity can't be higher than 1, 1 is used then\n"
                                          "Previous Active Fibers Quantity before set to 1: "
                                          + std::to_string(active));
        }
        resting += active - 1;

        active = 1;
    }

    // Sanity check for fatigued fibers
    if (fatigued < 0) {
        utils::Error::raise("Fatigued Fibers Quantity can't be lower than 0");
    } else if (fatigued > 1) {
        utils::Error::raise("Fatigued Fibers Quantity can't be higher than 1");
    }

    // Sanity check for resting fibers
    //
    // In order to get the quantity of resting fibers to 0, it has to come from the input command.
    // The input command manage fiber recruitment from resting to active.
    // Hence any exceeding because of integration can be corrected by putting back exceeding resting fibers quantity
    // into active fibers quantity.
    //
    if (resting < 0) {
        if (!turnOffWarnings) {
            utils::Error::warning(0,
                                          "Resting Fibers Quantity can't be lower than 0, 0 is used then\n"
                                          "Previous Resting Fibers Quantity before set to 0: "
                                          + std::to_string(resting));
        }
        active += resting;
        resting = 0;
    } else if (resting > 1) {
        utils::Error::raise(
            "Resting Fibers Quantity can't be higher than 1");

    }

    if (fabs(active + fatigued + resting - 1.0) > 0.1) {
        utils::Error::raise("Sum of the fatigued states must be equal to 1");
    }

    *m_activeFibers = active;
    *m_fatiguedFibers = fatigued;
    *m_restingFibers = resting;
}
#endif

const utils::Scalar& muscles::FatigueState::activeFibers() const
{
    return *m_activeFibers;
}

const utils::Scalar& muscles::FatigueState::fatiguedFibers()
const
{
    return *m_fatiguedFibers;
}

const utils::Scalar& muscles::FatigueState::restingFibers()
const
{
    return *m_restingFibers;
}

muscles::STATE_FATIGUE_TYPE muscles::FatigueState::getType()
const
{
    return *m_type;
}

void muscles::FatigueState::setType()
{
    *m_type = muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE;
}
