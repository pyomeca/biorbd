#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleStateDynamicsBuchanan.h"

s2mMuscleStateDynamicsBuchanan::s2mMuscleStateDynamicsBuchanan(const double &neuralCommand, const double &e) :
    s2mMuscleStateDynamics(e,0),
    m_neuralCommand(neuralCommand),
    m_shapeFactor(-3)
{
    // Update activation
    s2mMuscleStateDynamicsBuchanan::activation();
}

s2mMuscleStateDynamicsBuchanan::~s2mMuscleStateDynamicsBuchanan()
{
    //dtor
}

void s2mMuscleStateDynamicsBuchanan::shapeFactor(double shape_factor)
{
    m_shapeFactor = shape_factor;

    // Update activation
    s2mMuscleStateDynamicsBuchanan::activation();
}

double s2mMuscleStateDynamicsBuchanan::shapeFactor()
{
    return m_shapeFactor;
}

double s2mMuscleStateDynamicsBuchanan::timeDerivativeExcitation(const s2mMuscleCaracteristics &caract, const bool alreadyNormalized){
    // Move excitation to activation to use properly s2mMuscleStateDynamics::timeDerivativeActivation
    double activationTp = m_activation;
    m_activation = m_excitation;
    double excitationTp = m_excitation;
    m_excitation = m_neuralCommand;

    // Compute excitationDot
    m_excitationDot = s2mMuscleStateDynamics::timeDerivativeActivation(caract, alreadyNormalized);
    // Set back activationDot to 0 (since it is suppose to calculate excitationDot)
    m_activationDot = 0;
    m_excitation = excitationTp;
    m_activation = activationTp;

    return m_excitationDot;
}

void s2mMuscleStateDynamicsBuchanan::setExcitation(const double &val)
{
     s2mMuscleStateDynamics::setExcitation(val);

     // Update activation
     s2mMuscleStateDynamicsBuchanan::activation();
}

void s2mMuscleStateDynamicsBuchanan::setNeuralCommand(const double &val)
{
     m_neuralCommand = val;
}

double s2mMuscleStateDynamicsBuchanan::activation()
{
    double expShapeFactor(exp(m_shapeFactor));
    m_activation = (  pow(expShapeFactor, m_excitation) - 1) / (expShapeFactor - 1) ;

    return m_activation;
}

