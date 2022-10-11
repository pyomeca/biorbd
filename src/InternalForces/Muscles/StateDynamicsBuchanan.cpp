#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/StateDynamicsBuchanan.h"

#include <math.h>

using namespace BIORBD_NAMESPACE;
using namespace internalforce;

muscles::StateDynamicsBuchanan::StateDynamicsBuchanan(
    const utils::Scalar& neuralCommand,
    const utils::Scalar& excitation) :
    muscles::StateDynamics(excitation,0),
    m_neuralCommand(std::make_shared<utils::Scalar>(neuralCommand)),
    m_shapeFactor(std::make_shared<utils::Scalar>(-3)),
    m_excitationDot(std::make_shared<utils::Scalar>(0))
{
    setType();
    // Update activation
    muscles::StateDynamicsBuchanan::activation();
}

muscles::StateDynamicsBuchanan::StateDynamicsBuchanan(
    const muscles::State &other) :
    muscles::StateDynamics(other)
{
    const auto& buchanan =
        dynamic_cast<const muscles::StateDynamicsBuchanan&>(other);
    m_neuralCommand = buchanan.m_neuralCommand;
    m_shapeFactor = buchanan.m_shapeFactor;
    m_excitationDot = buchanan.m_excitationDot;
}

muscles::StateDynamicsBuchanan::~StateDynamicsBuchanan()
{
    //dtor
}

muscles::StateDynamicsBuchanan
muscles::StateDynamicsBuchanan::DeepCopy() const
{
    muscles::StateDynamicsBuchanan copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::StateDynamicsBuchanan::DeepCopy(
    const muscles::StateDynamicsBuchanan &other)
{
    muscles::StateDynamics::DeepCopy(other);
    *m_neuralCommand = *other.m_neuralCommand;
    *m_shapeFactor = *other.m_shapeFactor;
    *m_excitationDot = *other.m_excitationDot;
}

void muscles::StateDynamicsBuchanan::shapeFactor(
    const utils::Scalar& shape_factor)
{
    *m_shapeFactor = shape_factor;

    // Update activation
    setActivation(0);
}

const utils::Scalar&
muscles::StateDynamicsBuchanan::shapeFactor() const
{
    return *m_shapeFactor;
}

const utils::Scalar&
muscles::StateDynamicsBuchanan::timeDerivativeExcitation(
    const muscles::Characteristics &characteristics,
    bool alreadyNormalized)
{
    // Move excitation to activation to use properly muscles::StateDynamics::timeDerivativeActivation
    utils::Scalar activationTp = *m_activation;
    *m_activation = *m_excitation;
    utils::Scalar excitationTp = *m_excitation;
    *m_excitation = *m_neuralCommand;

    // Compute excitationDot
    *m_excitationDot = muscles::StateDynamics::timeDerivativeActivation(
                           characteristics, alreadyNormalized);
    // Set back activationDot to 0 (since it is suppose to calculate excitationDot)
    *m_activationDot = 0;
    *m_excitation = excitationTp;
    *m_activation = activationTp;

    return *m_excitationDot;
}

void muscles::StateDynamicsBuchanan::setExcitation(
    const utils::Scalar& val,
    bool)
{
    muscles::StateDynamics::setExcitation(val);

    // Update activation
    setActivation(0);
}

void muscles::StateDynamicsBuchanan::setNeuralCommand(
    const utils::Scalar& val)
{
    *m_neuralCommand = val;
}

void muscles::StateDynamicsBuchanan::setActivation(
    const utils::Scalar&,
    bool)
{
    utils::Scalar expShapeFactor(exp(*m_shapeFactor));
    *m_activation = (  pow(expShapeFactor,
                           *m_excitation) - 1) / (expShapeFactor - 1) ;
}

void muscles::StateDynamicsBuchanan::setType()
{
    *m_stateType = muscles::STATE_TYPE::BUCHANAN;
}

