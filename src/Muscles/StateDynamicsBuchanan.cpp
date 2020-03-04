#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamicsBuchanan.h"

#include <math.h>

biorbd::muscles::StateDynamicsBuchanan::StateDynamicsBuchanan(
        biorbd::utils::Scalar neuralCommand,
        biorbd::utils::Scalar excitation) :
    biorbd::muscles::StateDynamics(excitation,0),
    m_neuralCommand(std::make_shared<biorbd::utils::Scalar>(neuralCommand)),
    m_shapeFactor(std::make_shared<biorbd::utils::Scalar>(-3)),
    m_excitationDot(std::make_shared<biorbd::utils::Scalar>(0))
{
    setType();
    // Update activation
    biorbd::muscles::StateDynamicsBuchanan::activation();
}

biorbd::muscles::StateDynamicsBuchanan::StateDynamicsBuchanan(
        const biorbd::muscles::StateDynamicsBuchanan &other) :
    biorbd::muscles::StateDynamics(other),
    m_neuralCommand(other.m_neuralCommand),
    m_shapeFactor(other.m_shapeFactor),
    m_excitationDot(other.m_excitationDot)
{

}

biorbd::muscles::StateDynamicsBuchanan::~StateDynamicsBuchanan()
{
    //dtor
}

biorbd::muscles::StateDynamicsBuchanan biorbd::muscles::StateDynamicsBuchanan::DeepCopy() const
{
    biorbd::muscles::StateDynamicsBuchanan copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::StateDynamicsBuchanan::DeepCopy(
        const biorbd::muscles::StateDynamicsBuchanan &other)
{
    biorbd::muscles::StateDynamics::DeepCopy(other);
    *m_neuralCommand = *other.m_neuralCommand;
    *m_shapeFactor = *other.m_shapeFactor;
    *m_excitationDot = *other.m_excitationDot;
}

void biorbd::muscles::StateDynamicsBuchanan::shapeFactor(
        biorbd::utils::Scalar shape_factor)
{
    *m_shapeFactor = shape_factor;

    // Update activation
    setActivation(0);
}

biorbd::utils::Scalar biorbd::muscles::StateDynamicsBuchanan::shapeFactor() const
{
    return *m_shapeFactor;
}

biorbd::utils::Scalar biorbd::muscles::StateDynamicsBuchanan::timeDerivativeExcitation(
        const biorbd::muscles::Characteristics &characteristics,
        bool alreadyNormalized){
    // Move excitation to activation to use properly biorbd::muscles::StateDynamics::timeDerivativeActivation
    biorbd::utils::Scalar activationTp = *m_activation;
    *m_activation = *m_excitation;
    biorbd::utils::Scalar excitationTp = *m_excitation;
    *m_excitation = *m_neuralCommand;

    // Compute excitationDot
    *m_excitationDot = biorbd::muscles::StateDynamics::timeDerivativeActivation(characteristics, alreadyNormalized);
    // Set back activationDot to 0 (since it is suppose to calculate excitationDot)
    *m_activationDot = 0;
    *m_excitation = excitationTp;
    *m_activation = activationTp;

    return *m_excitationDot;
}

void biorbd::muscles::StateDynamicsBuchanan::setExcitation(
        biorbd::utils::Scalar  val,
        bool turnOffWarnings)
{
     biorbd::muscles::StateDynamics::setExcitation(val, turnOffWarnings);

     // Update activation
     setActivation(0);
}

void biorbd::muscles::StateDynamicsBuchanan::setNeuralCommand(
        biorbd::utils::Scalar val)
{
     *m_neuralCommand = val;
}

void biorbd::muscles::StateDynamicsBuchanan::setActivation(
        biorbd::utils::Scalar,
        bool)
{
    biorbd::utils::Scalar expShapeFactor(exp(*m_shapeFactor));
    *m_activation = (  pow(expShapeFactor, *m_excitation) - 1) / (expShapeFactor - 1) ;
}

void biorbd::muscles::StateDynamicsBuchanan::setType()
{
    *m_stateType = biorbd::muscles::STATE_TYPE::BUCHANAN;
}

