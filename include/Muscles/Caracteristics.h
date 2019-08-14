#ifndef BIORBD_MUSCLES_CARACTERISTICS_H
#define BIORBD_MUSCLES_CARACTERISTICS_H

#include <cstddef>
#include "biorbdConfig.h"
#include "Utils/Error.h"
#include "Muscles/State.h"
#include "Muscles/FatigueParameters.h"

namespace biorbd {
namespace  muscles {

class BIORBD_API Caracteristics
{
public:
    Caracteristics(
            const double &optLength = 0,
            const double &fmax = 0,
            const double &PCSA = 0,
            const double &tendonSlackLength = 0,
            const double &pennAngle = 0,
            const biorbd::muscles::State& stateMax = biorbd::muscles::State(),
            const biorbd::muscles::FatigueParameters& fatigueParameters = biorbd::muscles::FatigueParameters(),
            const double tauAct = 0.01,
            const double tauDeact = 0.04,
            const double &minAct =.01);
    Caracteristics(const Caracteristics&);
    Caracteristics& operator=(const Caracteristics&);
    virtual ~Caracteristics();

    // Get et Set
    virtual double optimalLength() const;
    double forceIsoMax() const;
    double tendonSlackLength() const;
    double pennationAngle() const;
    double PCSA() const;

    void minActivation(double val);
    double minActivation() const;
    void tauActivation(double val);
    double tauActivation() const;
    void tauDeactivation(double val);
    double tauDeactivation() const;

    void setOptimalLength(const double &val);
    virtual void setForceIsoMax(const double &val);
    void PCSA(const double &val);
    void setTendonSlackLength(const double &val);
    void setPennationAngle(const double &val);
    void setStateMax(const biorbd::muscles::State &stateMax);
    const biorbd::muscles::State& stateMax() const;

    const biorbd::muscles::FatigueParameters& fatigueParameters() const;
    void fatigueParameters(const biorbd::muscles::FatigueParameters& fatigueParameters);

protected:
    double m_optimalLength; // Longueur sans tension
    double m_fIsoMax;       // Force maximale isométrique
    double m_PCSA;          // PCSA du muscle
    double m_tendonSlackLength; // Tendon slack length
    double m_pennationAngle; // Angle de pennation
    biorbd::muscles::State m_stateMax; // Excitation et activation maximale du muscle

    // Parametre d'activation
    double m_minActivation; // Activation minimale
    double m_tauActivation; // Time activation constant
    double m_tauDeactivation; // Time deactivation constant

    // Fatigue parameters
    biorbd::muscles::FatigueParameters m_fatigueParameters; // Fatigue parameters
};

}}

#endif // BIORBD_MUSCLES_CARACTERISTICS_H
