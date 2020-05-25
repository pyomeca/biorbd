#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamicsDeGroote.h"

#include <math.h>
#include "Utils/Error.h"
#include "Utils/String.h"
#include "Muscles/Characteristics.h"

biorbd::muscles::StateDynamicsDeGroote::StateDynamicsDeGroote(
        const biorbd::utils::Scalar& excitation,
        const biorbd::utils::Scalar& activation) :
    biorbd::muscles::StateDynamics(excitation,activation)
{
    setType();
}

biorbd::muscles::StateDynamicsDeGroote::StateDynamicsDeGroote(
        const biorbd::muscles::StateDynamicsDeGroote &other) :
    biorbd::muscles::StateDynamics(other)
{

}

biorbd::muscles::StateDynamicsDeGroote biorbd::muscles::StateDynamicsDeGroote::DeepCopy() const
{
    biorbd::muscles::StateDynamicsDeGroote copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::StateDynamicsDeGroote::DeepCopy(const biorbd::muscles::StateDynamicsDeGroote &other)
{
    biorbd::muscles::StateDynamics::DeepCopy(other);
}

const biorbd::utils::Scalar& biorbd::muscles::StateDynamicsDeGroote::timeDerivativeActivation(
        const biorbd::muscles::Characteristics &characteristics,
        bool alreadyNormalized){
    // From DeGroote https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5043004/

    // Get activation and excitation
    if (*m_activation < characteristics.minActivation())
        *m_activation = characteristics.minActivation();

    if (*m_excitation < characteristics.minActivation())
        *m_excitation = characteristics.minActivation();


    biorbd::utils::Scalar diff; //(e - a)
    biorbd::utils::Scalar f;    //activation dynamics

    if (alreadyNormalized)
        diff = *m_excitation - *m_activation;
    else
        diff = normalizeExcitation(characteristics.stateMax())- *m_activation;
    f = 0.5 * tanh(0.1*diff);
    
    biorbd::utils::Scalar denom_activation;   // dénominateur for activation
    biorbd::utils::Scalar denom_deactivation; // dénominateur for deactivation

    denom_activation = characteristics.torqueActivation()   * (0.5+1.5* *m_activation);
    denom_deactivation = characteristics.torqueDeactivation() / (0.5+1.5* *m_activation);

    *m_activationDot = (((f+0.5)/denom_activation)+((-f + 0.5)/denom_deactivation))*diff;

    return *m_activationDot;
}

void biorbd::muscles::StateDynamicsDeGroote::setType()
{
    *m_stateType = biorbd::muscles::STATE_TYPE::DE_GROOTE;
}
