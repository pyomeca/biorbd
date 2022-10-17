#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/StateDynamicsDeGroote.h"

#include <math.h>
#include "Utils/Error.h"
#include "Utils/String.h"
#include "InternalForces/Muscles/Characteristics.h"

using namespace BIORBD_NAMESPACE;

internalforce::muscles::StateDynamicsDeGroote::StateDynamicsDeGroote(
    const utils::Scalar& excitation,
    const utils::Scalar& activation) :
    internalforce::muscles::StateDynamics(excitation,activation)
{
    setType();
}

internalforce::muscles::StateDynamicsDeGroote::StateDynamicsDeGroote(
    const internalforce::muscles::StateDynamicsDeGroote &other) :
    internalforce::muscles::StateDynamics(other)
{

}

internalforce::muscles::StateDynamicsDeGroote
internalforce::muscles::StateDynamicsDeGroote::DeepCopy() const
{
    internalforce::muscles::StateDynamicsDeGroote copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::StateDynamicsDeGroote::DeepCopy(const
        internalforce::muscles::StateDynamicsDeGroote &other)
{
    internalforce::muscles::StateDynamics::DeepCopy(other);
}

const utils::Scalar&
internalforce::muscles::StateDynamicsDeGroote::timeDerivativeActivation(
    const internalforce::muscles::Characteristics &characteristics,
    bool alreadyNormalized)
{
    // From DeGroote https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5043004/

    // Get activation and excitation and make difference
    utils::Scalar diff; //(e - a)
    utils::Scalar f;    //activation dynamics
    //std::tanh;

    diff = *m_excitation - *m_activation;
    f = 0.5 * tanh(0.1*diff);

    utils::Scalar denom_activation = characteristics.torqueActivation()
            * (0.5+1.5* *m_activation);
    utils::Scalar denom_deactivation =
        characteristics.torqueDeactivation() / (0.5+1.5* *m_activation);

    *m_activationDot = (((f+0.5)/denom_activation)+((-f + 0.5)/denom_deactivation))
                       *diff;
    return *m_activationDot;
}

void internalforce::muscles::StateDynamicsDeGroote::setType()
{
    *m_stateType = internalforce::muscles::STATE_TYPE::DE_GROOTE;
}
