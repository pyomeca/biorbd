#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/StateDynamicsBuchanan.h"

#include <math.h>

using namespace BIORBD_NAMESPACE;

internalforce::muscles::StateDynamicsBuchanan::StateDynamicsBuchanan(
    const utils::Scalar& neuralCommand,
    const utils::Scalar& excitation) :
    internalforce::muscles::StateDynamics(excitation,0),
    m_neuralCommand(std::make_shared<utils::Scalar>(neuralCommand)),
    m_shapeFactor(std::make_shared<utils::Scalar>(-3)),
    m_excitationDot(std::make_shared<utils::Scalar>(0))
{
    setType();
    // Update activation
    internalforce::muscles::StateDynamicsBuchanan::activation();
}

internalforce::muscles::StateDynamicsBuchanan::StateDynamicsBuchanan(
    const internalforce::muscles::State &other) :
    internalforce::muscles::StateDynamics(other)
{
    const auto& buchanan =
        dynamic_cast<const internalforce::muscles::StateDynamicsBuchanan&>(other);
    m_neuralCommand = buchanan.m_neuralCommand;
    m_shapeFactor = buchanan.m_shapeFactor;
    m_excitationDot = buchanan.m_excitationDot;
}

internalforce::muscles::StateDynamicsBuchanan::~StateDynamicsBuchanan()
{
    //dtor
}

internalforce::muscles::StateDynamicsBuchanan
internalforce::muscles::StateDynamicsBuchanan::DeepCopy() const
{
    internalforce::muscles::StateDynamicsBuchanan copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::StateDynamicsBuchanan::DeepCopy(
    const internalforce::muscles::StateDynamicsBuchanan &other)
{
    internalforce::muscles::StateDynamics::DeepCopy(other);
    *m_neuralCommand = *other.m_neuralCommand;
    *m_shapeFactor = *other.m_shapeFactor;
    *m_excitationDot = *other.m_excitationDot;
}

void internalforce::muscles::StateDynamicsBuchanan::shapeFactor(
    const utils::Scalar& shape_factor)
{
    *m_shapeFactor = shape_factor;

    // Update activation
    setActivation(0);
}

const utils::Scalar&
internalforce::muscles::StateDynamicsBuchanan::shapeFactor() const
{
    return *m_shapeFactor;
}

const utils::Scalar&
internalforce::muscles::StateDynamicsBuchanan::timeDerivativeExcitation(
    const internalforce::muscles::Characteristics &characteristics,
    bool alreadyNormalized)
{
    // Move excitation to activation to use properly internalforce::muscles::StateDynamics::timeDerivativeActivation
    utils::Scalar activationTp = *m_activation;
    *m_activation = *m_excitation;
    utils::Scalar excitationTp = *m_excitation;
    *m_excitation = *m_neuralCommand;

    // Compute excitationDot
    *m_excitationDot = internalforce::muscles::StateDynamics::timeDerivativeActivation(
                           characteristics, alreadyNormalized);
    // Set back activationDot to 0 (since it is suppose to calculate excitationDot)
    *m_activationDot = 0;
    *m_excitation = excitationTp;
    *m_activation = activationTp;

    return *m_excitationDot;
}

void internalforce::muscles::StateDynamicsBuchanan::setExcitation(
    const utils::Scalar& val,
    bool)
{
    internalforce::muscles::StateDynamics::setExcitation(val);

    // Update activation
    setActivation(0);
}

void internalforce::muscles::StateDynamicsBuchanan::setNeuralCommand(
    const utils::Scalar& val)
{
    *m_neuralCommand = val;
}

void internalforce::muscles::StateDynamicsBuchanan::setActivation(
    const utils::Scalar&,
    bool)
{
    utils::Scalar expShapeFactor(exp(*m_shapeFactor));
    *m_activation = (  pow(expShapeFactor,
                           *m_excitation) - 1) / (expShapeFactor - 1) ;
}

void internalforce::muscles::StateDynamicsBuchanan::setType()
{
    *m_stateType = internalforce::muscles::STATE_TYPE::BUCHANAN;
}

