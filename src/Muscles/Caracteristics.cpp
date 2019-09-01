#define BIORBD_API_EXPORTS
#include "Muscles/Caracteristics.h"

#include "Muscles/State.h"
#include "Muscles/FatigueParameters.h"

biorbd::muscles::Caracteristics::Caracteristics() :
    biorbd::utils::ShallowCopyObject (),
    m_optimalLength(std::make_shared<double>(0)),
    m_fIsoMax(std::make_shared<double>(0)),
    m_PCSA(std::make_shared<double>(0)),
    m_tendonSlackLength(std::make_shared<double>(0)),
    m_pennationAngle(std::make_shared<double>(0)),
    m_stateMax(std::make_shared<biorbd::muscles::State>(biorbd::muscles::State())),
    m_minActivation(std::make_shared<double>(0.01)),
    m_GeneralizedTorqueActivation(std::make_shared<double>(0.01)),
    m_GeneralizedTorqueDeactivation(std::make_shared<double>(0.04)),
    m_fatigueParameters(std::make_shared<biorbd::muscles::FatigueParameters>(biorbd::muscles::FatigueParameters()))
{

}

biorbd::muscles::Caracteristics::Caracteristics(const biorbd::muscles::Caracteristics &other) :
    biorbd::utils::ShallowCopyObject (other),
    m_optimalLength(other.m_optimalLength),
    m_fIsoMax(other.m_fIsoMax),
    m_PCSA(other.m_PCSA),
    m_tendonSlackLength(other.m_tendonSlackLength),
    m_pennationAngle(other.m_pennationAngle),
    m_stateMax(other.m_stateMax),
    m_minActivation(other.m_minActivation),
    m_GeneralizedTorqueActivation(other.m_GeneralizedTorqueActivation),
    m_GeneralizedTorqueDeactivation(other.m_GeneralizedTorqueDeactivation),
    m_fatigueParameters(other.m_fatigueParameters)
{

}

biorbd::muscles::Caracteristics::Caracteristics(
        double optLength,
        double fmax,
        double PCSA,
        double tendonSlackLength,
        double pennAngle,
        const biorbd::muscles::State &stateMax,
        const biorbd::muscles::FatigueParameters &fatigueParameters,
        double GeneralizedTorqueAct,
        double GeneralizedTorqueDeact,
        double minAct):
    biorbd::utils::ShallowCopyObject (),
    m_optimalLength(std::make_shared<double>(optLength)),
    m_fIsoMax(std::make_shared<double>(fmax)),
    m_PCSA(std::make_shared<double>(PCSA)),
    m_tendonSlackLength(std::make_shared<double>(tendonSlackLength)),
    m_pennationAngle(std::make_shared<double>(pennAngle)),
    m_stateMax(std::make_shared<biorbd::muscles::State>(stateMax)),
    m_minActivation(std::make_shared<double>(minAct)),
    m_GeneralizedTorqueActivation(std::make_shared<double>(GeneralizedTorqueAct)),
    m_GeneralizedTorqueDeactivation(std::make_shared<double>(GeneralizedTorqueDeact)),
    m_fatigueParameters(std::make_shared<biorbd::muscles::FatigueParameters>(fatigueParameters))
{

}

biorbd::muscles::Caracteristics::~Caracteristics()
{

}

biorbd::muscles::Caracteristics biorbd::muscles::Caracteristics::DeepCopy() const
{
    biorbd::muscles::Caracteristics copy;
    copy.DeepCopy(*this);
    return copy;
}
#include<iostream>
void biorbd::muscles::Caracteristics::DeepCopy(const biorbd::muscles::Caracteristics &other)
{
    std::cout << this->getObjectId() << std::endl;
    std::cout << other.getObjectId() << std::endl;
    *m_optimalLength = *other.m_optimalLength;
    *m_fIsoMax = *other.m_fIsoMax;
    *m_PCSA = *other.m_PCSA;
    *m_tendonSlackLength = *other.m_tendonSlackLength;
    *m_pennationAngle = *other.m_pennationAngle;
    *m_stateMax = other.m_stateMax->DeepCopy();
    *m_minActivation = *other.m_minActivation;
    *m_GeneralizedTorqueActivation = *other.m_GeneralizedTorqueActivation;
    *m_GeneralizedTorqueDeactivation = *other.m_GeneralizedTorqueDeactivation;
    *m_fatigueParameters = other.m_fatigueParameters->DeepCopy();
}

// Get et Set
double biorbd::muscles::Caracteristics::optimalLength() const
{
    return *m_optimalLength;
}
double biorbd::muscles::Caracteristics::forceIsoMax() const
{
    return *m_fIsoMax;
}
double biorbd::muscles::Caracteristics::tendonSlackLength() const
{
    return *m_tendonSlackLength;
}
double biorbd::muscles::Caracteristics::pennationAngle() const
{
    return *m_pennationAngle;
}
double biorbd::muscles::Caracteristics::PCSA() const
{
    return *m_PCSA;
}

void biorbd::muscles::Caracteristics::minActivation(double val)
{
    *m_minActivation = val;
}
double biorbd::muscles::Caracteristics::minActivation() const
{
    return *m_minActivation;
}
void biorbd::muscles::Caracteristics::GeneralizedTorqueActivation(double val)
{
    *m_GeneralizedTorqueActivation = val;
}
double biorbd::muscles::Caracteristics::GeneralizedTorqueActivation() const
{
    return *m_GeneralizedTorqueActivation;
}
void biorbd::muscles::Caracteristics::GeneralizedTorqueDeactivation(double val)
{
    *m_GeneralizedTorqueDeactivation = val;
}
double biorbd::muscles::Caracteristics::GeneralizedTorqueDeactivation() const
{
    return *m_GeneralizedTorqueDeactivation;
}


void biorbd::muscles::Caracteristics::setOptimalLength(double val)
{
    *m_optimalLength = val;
}
void biorbd::muscles::Caracteristics::setForceIsoMax(double val)
{
    *m_fIsoMax = val;
}
void biorbd::muscles::Caracteristics::PCSA(double val)
{
    *m_PCSA = val;
}
void biorbd::muscles::Caracteristics::setTendonSlackLength(double val)
{
    *m_tendonSlackLength = val;
}
void biorbd::muscles::Caracteristics::setPennationAngle(double val)
{
    *m_pennationAngle = val;
}

void biorbd::muscles::Caracteristics::setStateMax(const biorbd::muscles::State &stateMax) {
    *m_stateMax = stateMax;
}

const biorbd::muscles::State &biorbd::muscles::Caracteristics::stateMax() const {
    return *m_stateMax;
}

const biorbd::muscles::FatigueParameters &biorbd::muscles::Caracteristics::fatigueParameters() const
{
    return *m_fatigueParameters;
}

void biorbd::muscles::Caracteristics::fatigueParameters(const biorbd::muscles::FatigueParameters &fatigueParameters)
{
    *m_fatigueParameters = fatigueParameters;
}
