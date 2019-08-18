#define BIORBD_API_EXPORTS
#include "Muscles/Caracteristics.h"

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
    m_optimalLength(optLength),
    m_fIsoMax(fmax),
    m_PCSA(PCSA),
    m_tendonSlackLength(tendonSlackLength),
    m_pennationAngle(pennAngle),
    m_stateMax(stateMax),
    m_minActivation(minAct),
    m_GeneralizedTorqueActivation(GeneralizedTorqueAct),
    m_GeneralizedTorqueDeactivation(GeneralizedTorqueDeact),
    m_fatigueParameters(fatigueParameters)
{

}

// Get et Set
double biorbd::muscles::Caracteristics::optimalLength() const { return m_optimalLength; }
double biorbd::muscles::Caracteristics::forceIsoMax() const { return m_fIsoMax; }
double biorbd::muscles::Caracteristics::tendonSlackLength() const {return m_tendonSlackLength;}
double biorbd::muscles::Caracteristics::pennationAngle() const {return m_pennationAngle;}
double biorbd::muscles::Caracteristics::PCSA() const {return m_PCSA;}

void biorbd::muscles::Caracteristics::minActivation(double val){ m_minActivation = val;}
double biorbd::muscles::Caracteristics::minActivation() const { return m_minActivation;}
void biorbd::muscles::Caracteristics::GeneralizedTorqueActivation(double val){ m_GeneralizedTorqueActivation = val;}
double biorbd::muscles::Caracteristics::GeneralizedTorqueActivation() const { return m_GeneralizedTorqueActivation;}
void biorbd::muscles::Caracteristics::GeneralizedTorqueDeactivation(double val){ m_GeneralizedTorqueDeactivation = val;}
double biorbd::muscles::Caracteristics::GeneralizedTorqueDeactivation() const { return m_GeneralizedTorqueDeactivation;}


void biorbd::muscles::Caracteristics::setOptimalLength(double val) { m_optimalLength = val; }
void biorbd::muscles::Caracteristics::setForceIsoMax(double val) { m_fIsoMax = val; }
void biorbd::muscles::Caracteristics::PCSA(double val) {m_PCSA = val;}
void biorbd::muscles::Caracteristics::setTendonSlackLength(double val) {m_tendonSlackLength = val;}
void biorbd::muscles::Caracteristics::setPennationAngle(double val) {m_pennationAngle = val;}



biorbd::muscles::Caracteristics::Caracteristics(const biorbd::muscles::Caracteristics& c):
    m_optimalLength(c.m_optimalLength),
    m_fIsoMax(c.m_fIsoMax),
    m_PCSA(c.m_PCSA),
    m_tendonSlackLength(c.tendonSlackLength()),
    m_pennationAngle(c.m_pennationAngle),
    m_stateMax(c.m_stateMax),
    m_minActivation(c.m_minActivation),
    m_GeneralizedTorqueActivation(c.m_GeneralizedTorqueActivation),
    m_GeneralizedTorqueDeactivation(c.m_GeneralizedTorqueDeactivation),
    m_fatigueParameters(c.m_fatigueParameters)
{

}

biorbd::muscles::Caracteristics& biorbd::muscles::Caracteristics::operator=(const biorbd::muscles::Caracteristics& c){
    if (this==&c) // check for self-assigment
        return *this;

    m_optimalLength = c.m_optimalLength;
    m_fIsoMax = c.m_fIsoMax;
    m_PCSA = c.m_PCSA;
    m_tendonSlackLength = c.m_tendonSlackLength;
    m_pennationAngle = c.m_pennationAngle;
    setStateMax(c.m_stateMax);
    m_minActivation = c.m_minActivation;
    m_GeneralizedTorqueActivation = c.m_GeneralizedTorqueActivation;
    m_GeneralizedTorqueDeactivation = c.m_GeneralizedTorqueDeactivation;
    m_fatigueParameters = c.m_fatigueParameters;

    return *this;
}

biorbd::muscles::Caracteristics::~Caracteristics()
{

}

void biorbd::muscles::Caracteristics::setStateMax(const biorbd::muscles::State &stateMax) {
    m_stateMax = stateMax;
}

const biorbd::muscles::State &biorbd::muscles::Caracteristics::stateMax() const {
    return m_stateMax;
}

const biorbd::muscles::FatigueParameters &biorbd::muscles::Caracteristics::fatigueParameters() const
{
    return m_fatigueParameters;
}

void biorbd::muscles::Caracteristics::fatigueParameters(const biorbd::muscles::FatigueParameters &fatigueParameters)
{
    m_fatigueParameters = fatigueParameters;
}
