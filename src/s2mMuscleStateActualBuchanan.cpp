#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleStateActualBuchanan.h"

s2mMuscleStateActualBuchanan::s2mMuscleStateActualBuchanan(const double &neuralCommand, const double &e) :
    s2mMuscleStateActual(e,0),
    m_neuralCommand(neuralCommand),
    m_shapeFactor(-3)
{
    // Update activation
    s2mMuscleStateActualBuchanan::activation();
}

s2mMuscleStateActualBuchanan::~s2mMuscleStateActualBuchanan()
{
    //dtor
}

void s2mMuscleStateActualBuchanan::shapeFactor(double shape_factor)
{
    m_shapeFactor = shape_factor;

    // Update activation
    s2mMuscleStateActualBuchanan::activation();
}

double s2mMuscleStateActualBuchanan::shapeFactor()
{
    return m_shapeFactor;
}

double s2mMuscleStateActualBuchanan::timeDerivativeExcitation(const s2mMuscleCaracteristics &c, const bool alreadyNormalized){
	// Move excitation to activation to use properly s2mMuscleStateActual::timeDerivativeActivation
    double activationTp = m_activation;
    m_activation = m_excitation;
    double excitationTp = m_excitation;
    m_excitation = m_neuralCommand;

    // Compute excitationDot
    m_excitationDot = s2mMuscleStateActual::timeDerivativeActivation(c, alreadyNormalized);
    // Set back activationDot to 0 (since it is suppose to calculate excitationDot)
    m_activationDot = 0;
    m_excitation = excitationTp;
    m_activation = activationTp;

    return m_excitationDot;
}

void s2mMuscleStateActualBuchanan::setExcitation(const double &val)
{
     s2mMuscleStateActual::setExcitation(val);

     // Update activation
     s2mMuscleStateActualBuchanan::activation();
}

void s2mMuscleStateActualBuchanan::setNeuralCommand(const double &val)
{
     m_neuralCommand = val;
}

double s2mMuscleStateActualBuchanan::activation()
{
    double expShapeFactor(exp(m_shapeFactor));
    m_activation = (  pow(expShapeFactor, m_excitation) - 1) / (expShapeFactor - 1) ;

    return m_activation;
}

