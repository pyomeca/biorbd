#define BIORBD_API_EXPORTS
#include "Muscles/Characteristics.h"

#include "Muscles/State.h"
#include "Muscles/FatigueParameters.h"

biorbd::muscles::Characteristics::Characteristics() :
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

biorbd::muscles::Characteristics::Characteristics(const biorbd::muscles::Characteristics &other) :
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

biorbd::muscles::Characteristics::Characteristics(
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

biorbd::muscles::Characteristics::~Characteristics()
{

}

biorbd::muscles::Characteristics biorbd::muscles::Characteristics::DeepCopy() const
{
    biorbd::muscles::Characteristics copy;
    copy.DeepCopy(*this);
    return copy;
}
#include<iostream>
void biorbd::muscles::Characteristics::DeepCopy(const biorbd::muscles::Characteristics &other)
{
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
double biorbd::muscles::Characteristics::optimalLength() const
{
    return *m_optimalLength;
}
double biorbd::muscles::Characteristics::forceIsoMax() const
{
    return *m_fIsoMax;
}
double biorbd::muscles::Characteristics::tendonSlackLength() const
{
    return *m_tendonSlackLength;
}
double biorbd::muscles::Characteristics::pennationAngle() const
{
    return *m_pennationAngle;
}
double biorbd::muscles::Characteristics::PCSA() const
{
    return *m_PCSA;
}

void biorbd::muscles::Characteristics::minActivation(double val)
{
    *m_minActivation = val;
}
double biorbd::muscles::Characteristics::minActivation() const
{
    return *m_minActivation;
}
void biorbd::muscles::Characteristics::GeneralizedTorqueActivation(double val)
{
    *m_GeneralizedTorqueActivation = val;
}
double biorbd::muscles::Characteristics::GeneralizedTorqueActivation() const
{
    return *m_GeneralizedTorqueActivation;
}
void biorbd::muscles::Characteristics::GeneralizedTorqueDeactivation(double val)
{
    *m_GeneralizedTorqueDeactivation = val;
}
double biorbd::muscles::Characteristics::GeneralizedTorqueDeactivation() const
{
    return *m_GeneralizedTorqueDeactivation;
}


void biorbd::muscles::Characteristics::setOptimalLength(double val)
{
    *m_optimalLength = val;
}
void biorbd::muscles::Characteristics::setForceIsoMax(double val)
{
    *m_fIsoMax = val;
}
void biorbd::muscles::Characteristics::PCSA(double val)
{
    *m_PCSA = val;
}
void biorbd::muscles::Characteristics::setTendonSlackLength(double val)
{
    *m_tendonSlackLength = val;
}
void biorbd::muscles::Characteristics::setPennationAngle(double val)
{
    *m_pennationAngle = val;
}

void biorbd::muscles::Characteristics::setStateMax(const biorbd::muscles::State &stateMax) {
    *m_stateMax = stateMax;
}

const biorbd::muscles::State &biorbd::muscles::Characteristics::stateMax() const {
    return *m_stateMax;
}

const biorbd::muscles::FatigueParameters &biorbd::muscles::Characteristics::fatigueParameters() const
{
    return *m_fatigueParameters;
}

void biorbd::muscles::Characteristics::fatigueParameters(const biorbd::muscles::FatigueParameters &fatigueParameters)
{
    *m_fatigueParameters = fatigueParameters;
}
