#define BIORBD_API_EXPORTS
#include "s2mMuscleCaracteristics.h"


s2mMuscleCaracteristics::s2mMuscleCaracteristics(const double &optLength,
                                                 const double &fmax,
                                                 const double &PCSA,
                                                 const double &tendonSlackLength,
                                                 const double &pennAngle,
                                                 const s2mMuscleState &stateMax,
                                                 const double tauAct,
                                                 const double tauDeact,
                                                 const double &minAct
                                                 ):
    m_optimalLength(optLength),
    m_fIsoMax(fmax),
    m_PCSA(PCSA),
    m_tendonSlackLength(tendonSlackLength),
    m_pennationAngle(pennAngle),
    m_stateMax(stateMax),
    m_minActivation(minAct),
    m_tauActivation(tauAct),
    m_tauDeactivation(tauDeact)
{

}

// Get et Set
double s2mMuscleCaracteristics::optimalLength() const { return m_optimalLength; }
double s2mMuscleCaracteristics::forceIsoMax() const { return m_fIsoMax; }
double s2mMuscleCaracteristics::tendonSlackLength() const {return m_tendonSlackLength;}
double s2mMuscleCaracteristics::pennationAngle() const {return m_pennationAngle;}
double s2mMuscleCaracteristics::PCSA() const {return m_PCSA;}

void s2mMuscleCaracteristics::minActivation(double val){ m_minActivation = val;}
double s2mMuscleCaracteristics::minActivation() const { return m_minActivation;}
void s2mMuscleCaracteristics::tauActivation(double val){ m_tauActivation = val;}
double s2mMuscleCaracteristics::tauActivation() const { return m_tauActivation;}
void s2mMuscleCaracteristics::tauDeactivation(double val){ m_tauDeactivation = val;}
double s2mMuscleCaracteristics::tauDeactivation() const { return m_tauDeactivation;}


void s2mMuscleCaracteristics::setOptimalLength(const double &val) { m_optimalLength = val; }
void s2mMuscleCaracteristics::setForceIsoMax(const double &val) { m_fIsoMax = val; }
void s2mMuscleCaracteristics::PCSA(const double &val) {m_PCSA = val;}
void s2mMuscleCaracteristics::setTendonSlackLength(const double &val) {m_tendonSlackLength = val;}
void s2mMuscleCaracteristics::setPennationAngle(const double &val) {m_pennationAngle = val;}



s2mMuscleCaracteristics::s2mMuscleCaracteristics(const s2mMuscleCaracteristics& c):
    m_optimalLength(c.m_optimalLength),
    m_fIsoMax(c.m_fIsoMax),
    m_PCSA(c.m_PCSA),
    m_tendonSlackLength(c.tendonSlackLength()),
    m_pennationAngle(c.m_pennationAngle),
    m_stateMax(c.m_stateMax),
    m_minActivation(c.m_minActivation),
    m_tauActivation(c.m_tauActivation),
    m_tauDeactivation(c.m_tauDeactivation)
{

}

s2mMuscleCaracteristics& s2mMuscleCaracteristics::operator=(const s2mMuscleCaracteristics& c){
    if (this==&c) // check for self-assigment
        return *this;

    m_optimalLength = c.m_optimalLength;
    m_fIsoMax = c.m_fIsoMax;
    m_PCSA = c.m_PCSA;
    m_tendonSlackLength = c.m_tendonSlackLength;
    m_pennationAngle = c.m_pennationAngle;
    setStateMax(c.m_stateMax);
    m_minActivation = c.m_minActivation;
    m_tauActivation = c.m_tauActivation;
    m_tauDeactivation = c.m_tauDeactivation;

    return *this;
}

s2mMuscleCaracteristics::~s2mMuscleCaracteristics()
{

}

void s2mMuscleCaracteristics::setStateMax(const s2mMuscleState &stateMax) {
    m_stateMax = stateMax;
}

const s2mMuscleState &s2mMuscleCaracteristics::stateMax() const {
    return m_stateMax;
}
