#include "../include/s2mMuscleStateActualBuchanan.h"

s2mMuscleStateActualBuchanan::s2mMuscleStateActualBuchanan(const double &e, const double &a) :
    s2mMuscleStateActual(e,a),
    m_shape_factor(-3)
{
}

s2mMuscleStateActualBuchanan::~s2mMuscleStateActualBuchanan()
{
    //dtor
}

void s2mMuscleStateActualBuchanan::shapeFactor(double shape_factor)
{
    m_shape_factor = shape_factor;
}

double s2mMuscleStateActualBuchanan::shapeFactor()
{
    return m_shape_factor;
}

double s2mMuscleStateActualBuchanan::timeDerivativeActivation(const s2mMuscleCaracteristics &c, const bool alreadyNormalized){
    // Implémentation de la fonction da/dt = (u-a)/tau(u,a)
    // ou tau(u,a) = t_act(0.5+1.5*a) is u>a et tau(u,a)=t_deact(0.5+1.5*a) sinon
    if (m_activation<c.minActivation())
        m_activation = c.minActivation();

    if (m_excitation<c.minActivation())
        m_excitation = c.minActivation();



// see doi:10.1016/j.humov.2011.08.006
// see doi:10.1016/S0021-9290(03)00010-1


// http://simtk-confluence.stanford.edu:8080/display/OpenSim/First-Order+Activation+Dynamics

    double expShapeFactor(exp(m_shape_factor)); //Buchanan2004, le 22 mars 2018
    double m_excitation_shape;

    double num;
    if (alreadyNormalized){
        m_excitation_shape = (  pow(expShapeFactor,m_excitation) - 1) / (expShapeFactor - 1) ;
        num                = m_excitation_shape-m_activation;
        //num = m_excitation-m_activation; // numérateur
    }
    else
        num = (  pow(expShapeFactor,excitationNorm(c.stateMax())) - 1) / (expShapeFactor - 1) - m_activation; // numérateur
        //num = excitationNorm(c.stateMax())-m_activation; // numérateur

    double denom; // dénominateur
    if (num>0)
        denom = c.tauActivation()   * (0.5+1.5*m_activation);
    else
        denom = c.tauDeactivation() / (0.5+1.5*m_activation);

    m_activationDot = num/denom;

    return m_activationDot;
}

