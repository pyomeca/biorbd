#ifndef BIORBD_MUSCLES_CHARACTERISTICS_H
#define BIORBD_MUSCLES_CHARACTERISTICS_H

#include <memory>
#include <cstddef>
#include "biorbdConfig.h"

namespace biorbd {
namespace  muscles {
class State;
class FatigueParameters;
///
/// \brief Class Characterisitcs that holds that muscle characteristics
///
class BIORBD_API Characteristics
{
public:
    ///
    /// \brief Construct characteristics
    ///
    Characteristics();

    ///
    /// \brief Construct characteristics from other characteristics
    /// \param other The other characteristics
    ///
    Characteristics(const biorbd::muscles::Characteristics& other);

    ///
    /// \brief Construct characteristics
    /// \param optLength Length without tension
    /// \param fmax The maximal isometric force
    /// \param PCSA  Physiological cross-sectional area of the muscle 
    /// \param tendonSlackLength The tendon slack length
    /// \param pennAngle The angle of pennation
    /// \param stateMax Maximal excitation and activation of the muscle
    /// \param fatigueParameters The fatigue parameters
    /// \param GeneralizedTorqueAct Time activation constant (default: 0.01)
    /// \param GeneralizedTorqueDeact Time deactivation constant (default: 0.04)
    /// \param minAct Minimal activation (default: 0.01)
    ///
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

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Characteristics();

    ///
    /// \brief Deep copy of characteristics
    /// \return A depp copy of characteristics
    ///
    biorbd::muscles::Characteristics DeepCopy() const;

    ///
    /// \brief Deep copy of characteristics from another characteristics 
    /// \param other The characteristics to copy from
    ///
    void DeepCopy(const biorbd::muscles::Characteristics& other);


    ///
    /// \brief Return the length without tension
    /// \return The length without tension
    ///
    virtual double optimalLength() const;

    ///
    /// \brief Return the maximal isometric force
    /// \return The maximal isometric force
    ///    
    double forceIsoMax() const;

    ///
    /// \brief Return the tendon slack length
    /// \return The tendon slack length
    ///  
    double tendonSlackLength() const;

    ///
    /// \brief Return the angle of pennation
    /// \return The angle of pennation
    /// 
    double pennationAngle() const;

    ///
    /// \brief Return the physiological cross-sectional area of the muscle 
    /// \return The physiological cross-sectional area of the muscle 
    ///
    double PCSA() const;

    ///
    /// \brief Set the minimal activation of the muscle
    /// \param val The value of the minimal activation of the muscle
    ///
    void minActivation(double val);

    ///
    /// \brief Return the minimal activation of the muscle
    /// \return The minimal activation of the muscle
    ///
    double minActivation() const;

    ///
    /// \brief Set the time activation constant
    /// \param val The value of the time activation constant 
    ///
    void setGeneralizedTorqueActivation(double val);

    ///
    /// \brief Return the time activation constant
    /// \return The time activation constant
    ///
    double GeneralizedTorqueActivation() const;
    
    ///
    /// \brief Set the time deactivation constant
    /// \param val The value of the time deactivation constant
    ///
    void setGeneralizedTorqueDeactivation(double val);

    ///
    /// \brief Return the time deactivation constant
    /// \return The time deactivation constant
    ///
    double GeneralizedTorqueDeactivation() const;

    ///
    /// \brief Set the length without tension
    /// \param val Value of the length without tension
    ///
    void setOptimalLength(double val);

    ///
    /// \brief Set the maximal isometric force
    /// \param val Value of the maximal isometric force
    ///
    virtual void setForceIsoMax(double val);

    ///
    /// \brief Set the physiological cross-sectional area of the muscle
    /// \param val Value of the physiological cross-sectional area of the muscle
    ///
    void setPCSA(double val);

    ///
    /// \brief Set the tendon slack length
    /// \param val Value of the tendon slack length
    ///
    void setTendonSlackLength(double val);

    ///
    /// \brief Set the angle of pennation
    /// \param val Value of the angle of pennation
    ///
    void setPennationAngle(double val);

    ///
    /// \brief Set the maximal excitation and activation of the muscle
    /// \param stateMax Value of the maximal excitation and activation of the muscle
    ///
    void setStateMax(const biorbd::muscles::State &stateMax);

    ///
    /// \brief Return the maximal excitation and activation of the muscle
    /// \return The maximal excitation and activation of the muscle
    ///
    const biorbd::muscles::State& stateMax() const;

    ///
    /// \brief Return the fatigue parameters
    /// \return The fatigue parameters
    ///
    const biorbd::muscles::FatigueParameters& fatigueParameters() const;
    ///
    /// \brief Set the fatigue parameters
    /// \param fatigueParameters The values of the fatigue parameters
    ///
    void fatigueParameters(const biorbd::muscles::FatigueParameters& fatigueParameters);

protected:
    std::shared_ptr<double> m_optimalLength; ///< Length without tension
    std::shared_ptr<double> m_fIsoMax;       ///< Maximal isometric force Force maximale isométrique
    std::shared_ptr<double> m_PCSA;          ///< Physiological cross-sectional area of the muscle
    std::shared_ptr<double> m_tendonSlackLength; ///< Tendon slack length
    std::shared_ptr<double> m_pennationAngle; ///< Angle of pennation
    std::shared_ptr<biorbd::muscles::State> m_stateMax; ///< Maximal excitation et activation of the muscle

    // Parametre d'activation
    std::shared_ptr<double> m_minActivation; ///< Minimal activation 
    std::shared_ptr<double> m_GeneralizedTorqueActivation; ///<  Time activation constant
    std::shared_ptr<double> m_GeneralizedTorqueDeactivation; ///< Time deactivation constant

    // Fatigue parameters
    std::shared_ptr<biorbd::muscles::FatigueParameters> m_fatigueParameters; ///< Fatigue parameters
};

}}

#endif // BIORBD_MUSCLES_CHARACTERISTICS_H
