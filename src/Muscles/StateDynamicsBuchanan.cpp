#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamicsBuchanan.h"

#include <math.h>

biorbd::muscles::StateDynamicsBuchanan::StateDynamicsBuchanan(
        double neuralCommand,
        double e) :
    biorbd::muscles::StateDynamics(e,0),
    m_neuralCommand(neuralCommand),
    m_shapeFactor(-3)
{
    // Update activation
    biorbd::muscles::StateDynamicsBuchanan::activation();
}

biorbd::muscles::StateDynamicsBuchanan::~StateDynamicsBuchanan()
{
    //dtor
}

void biorbd::muscles::StateDynamicsBuchanan::shapeFactor(double shape_factor)
{
    m_shapeFactor = shape_factor;

    // Update activation
    biorbd::muscles::StateDynamicsBuchanan::activation();
}

double biorbd::muscles::StateDynamicsBuchanan::shapeFactor()
{
    return m_shapeFactor;
}

double biorbd::muscles::StateDynamicsBuchanan::timeDerivativeExcitation(
        const biorbd::muscles::Caracteristics &caract,
        bool alreadyNormalized){
    // Move excitation to activation to use properly biorbd::muscles::StateDynamics::timeDerivativeActivation
    double activationTp = m_activation;
    m_activation = m_excitation;
    double excitationTp = m_excitation;
    m_excitation = m_neuralCommand;

    // Compute excitationDot
    m_excitationDot = biorbd::muscles::StateDynamics::timeDerivativeActivation(caract, alreadyNormalized);
    // Set back activationDot to 0 (since it is suppose to calculate excitationDot)
    m_activationDot = 0;
    m_excitation = excitationTp;
    m_activation = activationTp;

    return m_excitationDot;
}

void biorbd::muscles::StateDynamicsBuchanan::setExcitation(double val)
{
     biorbd::muscles::StateDynamics::setExcitation(val);

     // Update activation
     biorbd::muscles::StateDynamicsBuchanan::activation();
}

void biorbd::muscles::StateDynamicsBuchanan::setNeuralCommand(double val)
{
     m_neuralCommand = val;
}

double biorbd::muscles::StateDynamicsBuchanan::activation()
{
    double expShapeFactor(exp(m_shapeFactor));
    m_activation = (  pow(expShapeFactor, m_excitation) - 1) / (expShapeFactor - 1) ;

    return m_activation;
}

void biorbd::muscles::StateDynamicsBuchanan::setType()
{
    m_stateType = biorbd::muscles::STATE_TYPE::BUCHANAN;
}

