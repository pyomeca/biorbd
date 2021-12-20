#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamicsDeGroote.h"

#include <math.h>
#include "Utils/Error.h"
#include "Utils/String.h"
#include "Muscles/Characteristics.h"

using namespace BIORBD_NAMESPACE;

muscles::StateDynamicsDeGroote::StateDynamicsDeGroote(
    const utils::Scalar& excitation,
    const utils::Scalar& activation) :
    muscles::StateDynamics(excitation,activation)
{
    setType();
}

muscles::StateDynamicsDeGroote::StateDynamicsDeGroote(
    const muscles::StateDynamicsDeGroote &other) :
    muscles::StateDynamics(other)
{

}

muscles::StateDynamicsDeGroote
muscles::StateDynamicsDeGroote::DeepCopy() const
{
    muscles::StateDynamicsDeGroote copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::StateDynamicsDeGroote::DeepCopy(const
        muscles::StateDynamicsDeGroote &other)
{
    muscles::StateDynamics::DeepCopy(other);
}

const utils::Scalar&
muscles::StateDynamicsDeGroote::timeDerivativeActivation(
    const muscles::Characteristics &characteristics,
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

void muscles::StateDynamicsDeGroote::setType()
{
    *m_stateType = muscles::STATE_TYPE::DE_GROOTE_STATE;
}
