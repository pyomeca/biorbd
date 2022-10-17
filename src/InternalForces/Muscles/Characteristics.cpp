#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/Characteristics.h"

#include "InternalForces/Muscles/State.h"
#include "InternalForces/Muscles/FatigueParameters.h"

using namespace BIORBD_NAMESPACE;

internalforce::muscles::Characteristics::Characteristics() :
    m_optimalLength(std::make_shared<utils::Scalar>(0)),
    m_fIsoMax(std::make_shared<utils::Scalar>(0)),
    m_PCSA(std::make_shared<utils::Scalar>(0)),
    m_tendonSlackLength(std::make_shared<utils::Scalar>(0)),
    m_pennationAngle(std::make_shared<utils::Scalar>(0)),
    m_stateMax(std::make_shared<internalforce::muscles::State>(internalforce::muscles::State(1, 1))),
    m_minActivation(std::make_shared<utils::Scalar>(0.01)),
    m_torqueActivation(std::make_shared<utils::Scalar>(0.01)),
    m_torqueDeactivation(std::make_shared<utils::Scalar>(0.04)),
    m_fatigueParameters(std::make_shared<internalforce::muscles::FatigueParameters>
                        (internalforce::muscles::FatigueParameters())),
    m_useDamping(std::make_shared<bool>(false))
{

}

internalforce::muscles::Characteristics::Characteristics(
    const internalforce::muscles::Characteristics &other) :
    m_optimalLength(other.m_optimalLength),
    m_fIsoMax(other.m_fIsoMax),
    m_PCSA(other.m_PCSA),
    m_tendonSlackLength(other.m_tendonSlackLength),
    m_pennationAngle(other.m_pennationAngle),
    m_stateMax(other.m_stateMax),
    m_minActivation(other.m_minActivation),
    m_torqueActivation(other.m_torqueActivation),
    m_torqueDeactivation(other.m_torqueDeactivation),
    m_fatigueParameters(other.m_fatigueParameters),
    m_useDamping(other.m_useDamping)
{

}

internalforce::muscles::Characteristics::Characteristics(
    const utils::Scalar& optLength,
    const utils::Scalar& fmax,
    const utils::Scalar& PCSA,
    const utils::Scalar& tendonSlackLength,
    const utils::Scalar& pennAngle,
    const internalforce::muscles::State &emgMax,
    const internalforce::muscles::FatigueParameters &fatigueParameters,
    bool useDamping,
    const utils::Scalar& torqueAct,
    const utils::Scalar& torqueDeact,
    const utils::Scalar& minAct):
    m_optimalLength(std::make_shared<utils::Scalar>(optLength)),
    m_fIsoMax(std::make_shared<utils::Scalar>(fmax)),
    m_PCSA(std::make_shared<utils::Scalar>(PCSA)),
    m_tendonSlackLength(std::make_shared<utils::Scalar>(tendonSlackLength)),
    m_pennationAngle(std::make_shared<utils::Scalar>(pennAngle)),
    m_stateMax(std::make_shared<internalforce::muscles::State>(emgMax)),
    m_minActivation(std::make_shared<utils::Scalar>(minAct)),
    m_torqueActivation(std::make_shared<utils::Scalar>(torqueAct)),
    m_torqueDeactivation(std::make_shared<utils::Scalar>(torqueDeact)),
    m_fatigueParameters(std::make_shared<internalforce::muscles::FatigueParameters>
                        (fatigueParameters)),
    m_useDamping(std::make_shared<bool>(useDamping))

{

}

internalforce::muscles::Characteristics::~Characteristics()
{

}

internalforce::muscles::Characteristics internalforce::muscles::Characteristics::DeepCopy()
const
{
    internalforce::muscles::Characteristics copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::Characteristics::DeepCopy(
    const internalforce::muscles::Characteristics &other)
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
    *m_useDamping = *other.m_useDamping;

}

// Get et Set
void internalforce::muscles::Characteristics::setOptimalLength(
    const utils::Scalar& val)
{
    *m_optimalLength = val;
}
const utils::Scalar& internalforce::muscles::Characteristics::optimalLength()
const
{
    return *m_optimalLength;
}

void internalforce::muscles::Characteristics::setForceIsoMax(
    const utils::Scalar& val)
{
    *m_fIsoMax = val;
}
const utils::Scalar& internalforce::muscles::Characteristics::forceIsoMax()
const
{
    return *m_fIsoMax;
}

void internalforce::muscles::Characteristics::setTendonSlackLength(
    const utils::Scalar& val)
{
    *m_tendonSlackLength = val;
}
const utils::Scalar&
internalforce::muscles::Characteristics::tendonSlackLength() const
{
    return *m_tendonSlackLength;
}

void internalforce::muscles::Characteristics::setPennationAngle(
    const utils::Scalar& val)
{
    *m_pennationAngle = val;
}
const utils::Scalar& internalforce::muscles::Characteristics::pennationAngle()
const
{
    return *m_pennationAngle;
}

void internalforce::muscles::Characteristics::setPCSA(
    const utils::Scalar& val)
{
    *m_PCSA = val;
}
const utils::Scalar& internalforce::muscles::Characteristics::PCSA() const
{
    return *m_PCSA;
}

void internalforce::muscles::Characteristics::setMinActivation(
    const utils::Scalar& val)
{
    *m_minActivation = val;
}
const utils::Scalar& internalforce::muscles::Characteristics::minActivation()
const
{
    return *m_minActivation;
}

void internalforce::muscles::Characteristics::setTorqueActivation(
    const utils::Scalar& val)
{
    *m_torqueActivation = val;
}
const utils::Scalar&
internalforce::muscles::Characteristics::torqueActivation() const
{
    return *m_torqueActivation;
}

void internalforce::muscles::Characteristics::setTorqueDeactivation(
    const utils::Scalar& val)
{
    *m_torqueDeactivation = val;
}
const utils::Scalar&
internalforce::muscles::Characteristics::torqueDeactivation() const
{
    return *m_torqueDeactivation;
}


void internalforce::muscles::Characteristics::setStateMax(
    const internalforce::muscles::State &emgMax)
{
    *m_stateMax = emgMax;
}
const internalforce::muscles::State &internalforce::muscles::Characteristics::stateMax() const
{
    return *m_stateMax;
}

void internalforce::muscles::Characteristics::setFatigueParameters(
    const internalforce::muscles::FatigueParameters &fatigueParameters)
{
    *m_fatigueParameters = fatigueParameters;
}
const internalforce::muscles::FatigueParameters
&internalforce::muscles::Characteristics::fatigueParameters() const
{
    return *m_fatigueParameters;
}

void internalforce::muscles::Characteristics::setUseDamping(
    bool val)
{
    *m_useDamping = val;
}
bool internalforce::muscles::Characteristics::useDamping() const
{
    return *m_useDamping;
}
