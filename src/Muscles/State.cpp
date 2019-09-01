#define BIORBD_API_EXPORTS
#include "Muscles/State.h"

biorbd::muscles::State::State(
        double excitation,
        double activation) :
    m_stateType(std::make_shared<biorbd::muscles::STATE_TYPE>()),
    m_excitation(std::make_shared<double>(excitation)),
    m_activation(std::make_shared<double>(activation))
{
    setType();
}

biorbd::muscles::State::State(
        const biorbd::muscles::State &other) :
    m_stateType(other.m_stateType),
    m_excitation(other.m_excitation),
    m_activation(other.m_activation)
{

}

biorbd::muscles::State::~State()
{
    //dtor
}

biorbd::muscles::State biorbd::muscles::State::DeepCopy() const
{
    biorbd::muscles::State copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::State::DeepCopy(const biorbd::muscles::State &other)
{
    *m_stateType = *other.m_stateType;
    *m_excitation = *other.m_excitation;
    *m_activation = *other.m_activation;
}

void biorbd::muscles::State::setExcitation(double val) {
    if (*m_excitation<=0)
        *m_excitation = 0;
    else
        *m_excitation = val;
}

void biorbd::muscles::State::setActivation(double val){
    if (*m_activation<=0)
        *m_activation = 0;
    else if (*m_activation>=1)
        *m_activation = 1;
    else
        *m_activation = val;
}

double biorbd::muscles::State::excitation() const
{
    return *m_excitation;
}

double biorbd::muscles::State::activation() const
{
    return *m_activation;
}

void biorbd::muscles::State::setType()
{
    *m_stateType = biorbd::muscles::STATE_TYPE::SIMPLE_STATE;
}
