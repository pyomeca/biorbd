#define BIORBD_API_EXPORTS
#include "Muscles/Characteristics.h"

#include "Muscles/State.h"
#include "Muscles/FatigueParameters.h"

using namespace BIORBD_NAMESPACE;

muscles::Characteristics::Characteristics() :
    m_optimalLength(std::make_shared<utils::Scalar>(0)),
    m_fIsoMax(std::make_shared<utils::Scalar>(0)),
    m_PCSA(std::make_shared<utils::Scalar>(0)),
    m_tendonSlackLength(std::make_shared<utils::Scalar>(0)),
    m_pennationAngle(std::make_shared<utils::Scalar>(0)),
    m_stateMax(std::make_shared<muscles::State>(muscles::State(1, 1))),
    m_minActivation(std::make_shared<utils::Scalar>(0.01)),
    m_torqueActivation(std::make_shared<utils::Scalar>(0.01)),
    m_torqueDeactivation(std::make_shared<utils::Scalar>(0.04)),
    m_fatigueParameters(std::make_shared<muscles::FatigueParameters>
                        (muscles::FatigueParameters()))
{

}

muscles::Characteristics::Characteristics(
    const muscles::Characteristics &other) :
    m_optimalLength(other.m_optimalLength),
    m_fIsoMax(other.m_fIsoMax),
    m_PCSA(other.m_PCSA),
    m_tendonSlackLength(other.m_tendonSlackLength),
    m_pennationAngle(other.m_pennationAngle),
    m_stateMax(other.m_stateMax),
    m_minActivation(other.m_minActivation),
    m_torqueActivation(other.m_torqueActivation),
    m_torqueDeactivation(other.m_torqueDeactivation),
    m_fatigueParameters(other.m_fatigueParameters)
{

}

muscles::Characteristics::Characteristics(
    const utils::Scalar& optLength,
    const utils::Scalar& fmax,
    const utils::Scalar& PCSA,
    const utils::Scalar& tendonSlackLength,
    const utils::Scalar& pennAngle,
    const muscles::State &emgMax,
    const muscles::FatigueParameters &fatigueParameters,
    const utils::Scalar& torqueAct,
    const utils::Scalar& torqueDeact,
    const utils::Scalar& minAct):
    m_optimalLength(std::make_shared<utils::Scalar>(optLength)),
    m_fIsoMax(std::make_shared<utils::Scalar>(fmax)),
    m_PCSA(std::make_shared<utils::Scalar>(PCSA)),
    m_tendonSlackLength(std::make_shared<utils::Scalar>(tendonSlackLength)),
    m_pennationAngle(std::make_shared<utils::Scalar>(pennAngle)),
    m_stateMax(std::make_shared<muscles::State>(emgMax)),
    m_minActivation(std::make_shared<utils::Scalar>(minAct)),
    m_torqueActivation(std::make_shared<utils::Scalar>(torqueAct)),
    m_torqueDeactivation(std::make_shared<utils::Scalar>(torqueDeact)),
    m_fatigueParameters(std::make_shared<muscles::FatigueParameters>
                        (fatigueParameters))
{

}

muscles::Characteristics::~Characteristics()
{

}

muscles::Characteristics muscles::Characteristics::DeepCopy()
const
{
    muscles::Characteristics copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::Characteristics::DeepCopy(
    const muscles::Characteristics &other)
{
    *m_optimalLength = *other.m_optimalLength;
    *m_fIsoMax = *other.m_fIsoMax;
    *m_PCSA = *other.m_PCSA;
    *m_tendonSlackLength = *other.m_tendonSlackLength;
    *m_pennationAngle = *other.m_pennationAngle;
    *m_stateMax = other.m_stateMax->DeepCopy();
    *m_minActivation = *other.m_minActivation;
    *m_torqueActivation = *other.m_torqueActivation;
    *m_torqueDeactivation = *other.m_torqueDeactivation;
    *m_fatigueParameters = other.m_fatigueParameters->DeepCopy();
}

// Get et Set
void muscles::Characteristics::setOptimalLength(
    const utils::Scalar& val)
{
    *m_optimalLength = val;
}
const utils::Scalar& muscles::Characteristics::optimalLength()
const
{
    return *m_optimalLength;
}

void muscles::Characteristics::setForceIsoMax(
    const utils::Scalar& val)
{
    *m_fIsoMax = val;
}
const utils::Scalar& muscles::Characteristics::forceIsoMax()
const
{
    return *m_fIsoMax;
}

void muscles::Characteristics::setTendonSlackLength(
    const utils::Scalar& val)
{
    *m_tendonSlackLength = val;
}
const utils::Scalar&
muscles::Characteristics::tendonSlackLength() const
{
    return *m_tendonSlackLength;
}

void muscles::Characteristics::setPennationAngle(
    const utils::Scalar& val)
{
    *m_pennationAngle = val;
}
const utils::Scalar& muscles::Characteristics::pennationAngle()
const
{
    return *m_pennationAngle;
}

void muscles::Characteristics::setPCSA(
    const utils::Scalar& val)
{
    *m_PCSA = val;
}
const utils::Scalar& muscles::Characteristics::PCSA() const
{
    return *m_PCSA;
}

void muscles::Characteristics::setMinActivation(
    const utils::Scalar& val)
{
    *m_minActivation = val;
}
const utils::Scalar& muscles::Characteristics::minActivation()
const
{
    return *m_minActivation;
}

void muscles::Characteristics::setTorqueActivation(
    const utils::Scalar& val)
{
    *m_torqueActivation = val;
}
const utils::Scalar&
muscles::Characteristics::torqueActivation() const
{
    return *m_torqueActivation;
}

void muscles::Characteristics::setTorqueDeactivation(
    const utils::Scalar& val)
{
    *m_torqueDeactivation = val;
}
const utils::Scalar&
muscles::Characteristics::torqueDeactivation() const
{
    return *m_torqueDeactivation;
}


void muscles::Characteristics::setStateMax(
    const muscles::State &emgMax)
{
    *m_stateMax = emgMax;
}
const muscles::State &muscles::Characteristics::stateMax() const
{
    return *m_stateMax;
}

void muscles::Characteristics::setFatigueParameters(
    const muscles::FatigueParameters &fatigueParameters)
{
    *m_fatigueParameters = fatigueParameters;
}
const muscles::FatigueParameters
&muscles::Characteristics::fatigueParameters() const
{
    return *m_fatigueParameters;
}

