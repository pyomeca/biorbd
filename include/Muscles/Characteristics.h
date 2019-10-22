#ifndef BIORBD_MUSCLES_CHARACTERISTICS_H
#define BIORBD_MUSCLES_CHARACTERISTICS_H

#include <memory>
#include <cstddef>
#include "biorbdConfig.h"

namespace biorbd {
namespace  muscles {
class State;
class FatigueParameters;

class BIORBD_API Characteristics
{
public:
    Characteristics();
    Characteristics(const biorbd::muscles::Characteristics& other);
    Characteristics(
            double optLength,
            double fmax,
            double PCSA,
            double tendonSlackLength,
            double pennAngle,
            const biorbd::muscles::State& stateMax,
            const biorbd::muscles::FatigueParameters& fatigueParameters,
            double GeneralizedTorqueAct = 0.01,
            double GeneralizedTorqueDeact = 0.04,
            double minAct = 0.01);
    virtual ~Characteristics();
    biorbd::muscles::Characteristics DeepCopy() const;
    void DeepCopy(const biorbd::muscles::Characteristics& other);

    // Get et Set
    virtual double optimalLength() const;
    double forceIsoMax() const;
    double tendonSlackLength() const;
    double pennationAngle() const;
    double PCSA() const;

    void minActivation(double val);
    double minActivation() const;
    void GeneralizedTorqueActivation(double val);
    double GeneralizedTorqueActivation() const;
    void GeneralizedTorqueDeactivation(double val);
    double GeneralizedTorqueDeactivation() const;

    void setOptimalLength(double val);
    virtual void setForceIsoMax(double val);
    void PCSA(double val);
    void setTendonSlackLength(double val);
    void setPennationAngle(double val);
    void setStateMax(const biorbd::muscles::State &stateMax);
    const biorbd::muscles::State& stateMax() const;

    const biorbd::muscles::FatigueParameters& fatigueParameters() const;
    void fatigueParameters(const biorbd::muscles::FatigueParameters& fatigueParameters);

protected:
    std::shared_ptr<double> m_optimalLength; // Longueur sans tension
    std::shared_ptr<double> m_fIsoMax;       // Force maximale isométrique
    std::shared_ptr<double> m_PCSA;          // PCSA du muscle
    std::shared_ptr<double> m_tendonSlackLength; // Tendon slack length
    std::shared_ptr<double> m_pennationAngle; // Angle de pennation
    std::shared_ptr<biorbd::muscles::State> m_stateMax; // Excitation et activation maximale du muscle

    // Parametre d'activation
    std::shared_ptr<double> m_minActivation; // Activation minimale
    std::shared_ptr<double> m_GeneralizedTorqueActivation; // Time activation constant
    std::shared_ptr<double> m_GeneralizedTorqueDeactivation; // Time deactivation constant

    // Fatigue parameters
    std::shared_ptr<biorbd::muscles::FatigueParameters> m_fatigueParameters; // Fatigue parameters
};

}}

#endif // BIORBD_MUSCLES_CHARACTERISTICS_H
