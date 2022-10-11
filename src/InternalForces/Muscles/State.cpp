#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/State.h"

#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;
using namespace internalforce;

muscles::State::State(
    const utils::Scalar& excitation,
    const utils::Scalar& activation) :
    m_stateType(std::make_shared<muscles::STATE_TYPE>()),
    m_excitation(std::make_shared<utils::Scalar>(excitation)),
    m_excitationNorm(std::make_shared<utils::Scalar>(0)),
    m_activation(std::make_shared<utils::Scalar>(activation))
{
    setType();
}

muscles::State::State(
    const muscles::State &other) :
    m_stateType(other.m_stateType),
    m_excitation(other.m_excitation),
    m_excitationNorm(std::make_shared<utils::Scalar>(0)),
    m_activation(other.m_activation)
{

}

muscles::State::~State()
{
    //dtor
}

muscles::State muscles::State::DeepCopy() const
{
    muscles::State copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::State::DeepCopy(const muscles::State &other)
{
    *m_stateType = *other.m_stateType;
    *m_excitation = *other.m_excitation;
    *m_excitationNorm = *other.m_excitationNorm;
    *m_activation = *other.m_activation;
}

void muscles::State::setExcitation(
    const utils::Scalar& val,
    bool turnOffWarnings)
{

#ifdef BIORBD_USE_CASADI_MATH
    *m_excitation = val;
#else
    if (val<0) {
        if (!turnOffWarnings) {
            utils::Error::warning(
                0, "Excitation can't be lower than 0, 0 is used then");
        }
        *m_excitation = 0;
    } else {
        *m_excitation = val;
    }
#endif
}

const utils::Scalar& muscles::State::excitation() const
{
    return *m_excitation;
}

const utils::Scalar& muscles::State::normalizeExcitation(
    const muscles::State &emgMax,
    bool turnOffWarnings)
{

#ifndef BIORBD_USE_CASADI_MATH
    if (!turnOffWarnings) {
        utils::Error::warning(
            *m_excitation < emgMax.excitation(),
            "Excitation is higher than maximal excitation.");
    }
#endif
    *m_excitationNorm = *m_excitation / emgMax.excitation();

    return *m_excitationNorm;
}

void muscles::State::setExcitationNorm(
    const utils::Scalar& val)
{
    *m_excitationNorm = val;
}

const utils::Scalar& muscles::State::excitationNorm() const
{
    return *m_excitationNorm;
}

void muscles::State::setActivation(
    const utils::Scalar& val,
    bool turnOffWarnings)
{
#ifdef BIORBD_USE_CASADI_MATH
    *m_activation = val;
#else
    if (val <= 0) {
        if (!turnOffWarnings) {
            utils::Error::warning(
                0, "Activation is " + utils::String::to_string(val) +
                " but can't be lower than 0, 0 is used then");
        }
        *m_activation = 0;
    } else if (val >= 1) {
        if (!turnOffWarnings) {
            utils::Error::warning(
                0, "Activation " + utils::String::to_string(val) +
                " but can't be higher than 1, 1 is used then");
        }
        *m_activation = 1;
    } else {
        *m_activation = val;
    }
#endif
}

const utils::Scalar& muscles::State::activation() const
{
    return *m_activation;
}

muscles::STATE_TYPE muscles::State::type() const
{
    return *m_stateType;
}

void muscles::State::setType()
{
    *m_stateType = muscles::STATE_TYPE::SIMPLE_STATE;
}
