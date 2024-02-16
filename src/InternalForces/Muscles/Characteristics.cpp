#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/Characteristics.h"

#include "InternalForces/Muscles/State.h"
#include "InternalForces/Muscles/FatigueParameters.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::Characteristics::Characteristics() :
    m_optimalLength(std::make_shared<utils::Scalar>(0)),
    m_fIsoMax(std::make_shared<utils::Scalar>(0)),
    m_PCSA(std::make_shared<utils::Scalar>(0)),
    m_tendonSlackLength(std::make_shared<utils::Scalar>(0)),
    m_pennationAngle(std::make_shared<utils::Scalar>(0)),
    m_stateMax(std::make_shared<internal_forces::muscles::State>(internal_forces::muscles::State(1, 1))),
    m_minActivation(std::make_shared<utils::Scalar>(0.01)),
    m_torqueActivation(std::make_shared<utils::Scalar>(0.01)),
    m_torqueDeactivation(std::make_shared<utils::Scalar>(0.04)),
    m_fatigueParameters(std::make_shared<internal_forces::muscles::FatigueParameters>
                        (internal_forces::muscles::FatigueParameters())),
    m_useDamping(std::make_shared<bool>(false)),
    m_maxShorteningSpeed(std::make_shared<utils::Scalar>(10.0))
{

}

internal_forces::muscles::Characteristics::Characteristics(
    const internal_forces::muscles::Characteristics &other) :
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
    m_useDamping(other.m_useDamping),
    m_maxShorteningSpeed(other.m_maxShorteningSpeed)
{

}

internal_forces::muscles::Characteristics::Characteristics(
    const utils::Scalar& optLength,
    const utils::Scalar& fmax,
    const utils::Scalar& PCSA,
    const utils::Scalar& tendonSlackLength,
    const utils::Scalar& pennAngle,
    const internal_forces::muscles::State &emgMax,
    const internal_forces::muscles::FatigueParameters &fatigueParameters,
    bool useDamping,
    const utils::Scalar& maxShorteningSpeed,
    const utils::Scalar& torqueAct,
    const utils::Scalar& torqueDeact,
    const utils::Scalar& minAct
    ):
    m_optimalLength(std::make_shared<utils::Scalar>(optLength)),
    m_fIsoMax(std::make_shared<utils::Scalar>(fmax)),
    m_PCSA(std::make_shared<utils::Scalar>(PCSA)),
    m_tendonSlackLength(std::make_shared<utils::Scalar>(tendonSlackLength)),
    m_pennationAngle(std::make_shared<utils::Scalar>(pennAngle)),
    m_stateMax(std::make_shared<internal_forces::muscles::State>(emgMax)),
    m_minActivation(std::make_shared<utils::Scalar>(minAct)),
    m_torqueActivation(std::make_shared<utils::Scalar>(torqueAct)),
    m_torqueDeactivation(std::make_shared<utils::Scalar>(torqueDeact)),
    m_fatigueParameters(std::make_shared<internal_forces::muscles::FatigueParameters>(fatigueParameters)),
    m_useDamping(std::make_shared<bool>(useDamping)),
    m_maxShorteningSpeed(std::make_shared<utils::Scalar>(maxShorteningSpeed))
{

}

internal_forces::muscles::Characteristics::~Characteristics()
{

}

internal_forces::muscles::Characteristics internal_forces::muscles::Characteristics::DeepCopy()
const
{
    internal_forces::muscles::Characteristics copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::Characteristics::DeepCopy(
    const internal_forces::muscles::Characteristics &other)
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
    *m_maxShorteningSpeed = *other.m_maxShorteningSpeed;

}

// Get et Set
void internal_forces::muscles::Characteristics::setOptimalLength(
    const utils::Scalar& val)
{
    *m_optimalLength = val;
}
const utils::Scalar& internal_forces::muscles::Characteristics::optimalLength()
const
{
    return *m_optimalLength;
}

void internal_forces::muscles::Characteristics::setForceIsoMax(
    const utils::Scalar& val)
{
    *m_fIsoMax = val;
}
const utils::Scalar& internal_forces::muscles::Characteristics::forceIsoMax()
const
{
    return *m_fIsoMax;
}

void internal_forces::muscles::Characteristics::setTendonSlackLength(
    const utils::Scalar& val)
{
    *m_tendonSlackLength = val;
}
const utils::Scalar&
internal_forces::muscles::Characteristics::tendonSlackLength() const
{
    return *m_tendonSlackLength;
}

void internal_forces::muscles::Characteristics::setPennationAngle(
    const utils::Scalar& val)
{
    *m_pennationAngle = val;
}
const utils::Scalar& internal_forces::muscles::Characteristics::pennationAngle()
const
{
    return *m_pennationAngle;
}

void internal_forces::muscles::Characteristics::setPCSA(
    const utils::Scalar& val)
{
    *m_PCSA = val;
}
const utils::Scalar& internal_forces::muscles::Characteristics::PCSA() const
{
    return *m_PCSA;
}

void internal_forces::muscles::Characteristics::setMinActivation(
    const utils::Scalar& val)
{
    *m_minActivation = val;
}
const utils::Scalar& internal_forces::muscles::Characteristics::minActivation()
const
{
    return *m_minActivation;
}

void internal_forces::muscles::Characteristics::setMaxShorteningSpeed(
    const utils::Scalar& val)
{
    *m_maxShorteningSpeed = val;
}

const utils::Scalar& internal_forces::muscles::Characteristics::maxShorteningSpeed() const
{
    return *m_maxShorteningSpeed;
}

void internal_forces::muscles::Characteristics::setTorqueActivation(
    const utils::Scalar& val)
{
    *m_torqueActivation = val;
}
const utils::Scalar&
internal_forces::muscles::Characteristics::torqueActivation() const
{
    return *m_torqueActivation;
}

void internal_forces::muscles::Characteristics::setTorqueDeactivation(
    const utils::Scalar& val)
{
    *m_torqueDeactivation = val;
}
const utils::Scalar&
internal_forces::muscles::Characteristics::torqueDeactivation() const
{
    return *m_torqueDeactivation;
}


void internal_forces::muscles::Characteristics::setStateMax(
    const internal_forces::muscles::State &emgMax)
{
    *m_stateMax = emgMax;
}
const internal_forces::muscles::State &internal_forces::muscles::Characteristics::stateMax() const
{
    return *m_stateMax;
}

void internal_forces::muscles::Characteristics::setFatigueParameters(
    const internal_forces::muscles::FatigueParameters &fatigueParameters)
{
    *m_fatigueParameters = fatigueParameters;
}
const internal_forces::muscles::FatigueParameters
&internal_forces::muscles::Characteristics::fatigueParameters() const
{
    return *m_fatigueParameters;
}

void internal_forces::muscles::Characteristics::setUseDamping(
    bool val)
{
    *m_useDamping = val;
}
bool internal_forces::muscles::Characteristics::useDamping() const
{
    return *m_useDamping;
}
