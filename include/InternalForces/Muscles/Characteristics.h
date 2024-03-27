#ifndef BIORBD_MUSCLES_CHARACTERISTICS_H
#define BIORBD_MUSCLES_CHARACTERISTICS_H

#include <memory>
#include <cstddef>
#include "biorbdConfig.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
namespace  muscles
{
class State;
class FatigueParameters;
///
/// \brief Class Holds that muscle characteristics
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
    Characteristics(
        const Characteristics& other);

    ///
    /// \brief Construct characteristics
    /// \param optLength Optimal length (where maximal force occurs)
    /// \param fmax The maximal isometric force at optimal length
    /// \param PCSA Physiological cross-sectional area of the muscle
    /// \param tendonSlackLength The tendon slack length
    /// \param pennAngle The angle of pennation
    /// \param emgMax Maximal excitation and activation of the muscle
    /// \param fatigueParameters The fatigue model
    /// \param torqueAct Time activation constant (default: 0.01)
    /// \param torqueDeact Time deactivation constant (default: 0.04)
    /// \param minAct Minimal activation (default: 0.01)
    /// \param useDamping Use damping (default: false)
    /// \param maxShorteningSpeed Maximal velocity of shortening (default: 10.0)
    ///
    Characteristics(
        const utils::Scalar& optLength,
        const utils::Scalar& fmax,
        const utils::Scalar& PCSA,
        const utils::Scalar& tendonSlackLength,
        const utils::Scalar& pennAngle,
        const State& emgMax,
        const FatigueParameters& fatigueParameters,
        bool useDamping=false,
        const utils::Scalar& maxShorteningSpeed = 10.0,
        const utils::Scalar& torqueAct = 0.01,
        const utils::Scalar& torqueDeact = 0.04,
        const utils::Scalar& minAct = 0.01
    );

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Characteristics();

    ///
    /// \brief Deep copy of characteristics
    /// \return A depp copy of characteristics
    ///
    Characteristics DeepCopy() const;

    ///
    /// \brief Deep copy of characteristics from another characteristics
    /// \param other The characteristics to copy from
    ///
    void DeepCopy(
        const Characteristics& other);


    ///
    /// \brief Set the length without tension
    /// \param val Value of the length without tension
    ///
    void setOptimalLength(
        const utils::Scalar& val);

    ///
    /// \brief Return the optimal length at which maximal force occurs
    /// \return The length without tension
    ///
    virtual const utils::Scalar& optimalLength() const;

    ///
    /// \brief Set the maximal isometric force
    /// \param val Value of the maximal isometric force
    ///
    virtual void setForceIsoMax(
        const utils::Scalar& val);

    ///
    /// \brief Return the maximal isometric force at optimal length
    /// \return The maximal isometric force
    ///
    const utils::Scalar& forceIsoMax() const;

    ///
    /// \brief Set the tendon slack length
    /// \param val Value of the tendon slack length
    ///
    void setTendonSlackLength(
        const utils::Scalar& val);
    ///
    /// \brief Return the tendon slack length
    /// \return The tendon slack length
    ///
    const utils::Scalar& tendonSlackLength() const;

    ///
    /// \brief Set the angle of pennation
    /// \param val Value of the angle of pennation
    ///
    void setPennationAngle(
        const utils::Scalar& val);
    ///
    /// \brief Return the angle of pennation
    /// \return The angle of pennation
    ///
    const utils::Scalar& pennationAngle() const;

    ///
    /// \brief Set the physiological cross-sectional area of the muscle
    /// \param val Value of the physiological cross-sectional area of the muscle
    ///
    void setPCSA(
        const utils::Scalar& val);

    ///
    /// \brief Return the physiological cross-sectional area of the muscle
    /// \return The physiological cross-sectional area of the muscle
    ///
    const utils::Scalar& PCSA() const;

    ///
    /// \brief Set the minimal activation of the muscle
    /// \param val The value of the minimal activation of the muscle
    ///
    void setMinActivation(
        const utils::Scalar& val);

    ///
    /// \brief Return the minimal activation of the muscle
    /// \return The minimal activation of the muscle
    ///
    const utils::Scalar& minActivation() const;

    ///
    /// \brief Set the maximal velocity of shortening
    /// \param val The value of the maximal velocity of shortening
    ///
    void setMaxShorteningSpeed(
        const utils::Scalar& val);

    ///
    /// \brief Return the maximal velocity of shortening
    /// \return The maximal velocity of shortening
    ///
    const utils::Scalar& maxShorteningSpeed() const;

    ///
    /// \brief Set the time activation constant
    /// \param val The value of the time activation constant
    ///
    void setTorqueActivation(
        const utils::Scalar& val);

    ///
    /// \brief Return the time activation constant
    /// \return The time activation constant
    ///
    const utils::Scalar& torqueActivation() const;

    ///
    /// \brief Set the time deactivation constant
    /// \param val The value of the time deactivation constant
    ///
    void setTorqueDeactivation(
        const utils::Scalar& val);

    ///
    /// \brief Return the time deactivation constant
    /// \return The time deactivation constant
    ///
    const utils::Scalar& torqueDeactivation() const;

    ///
    /// \brief Set the maximal excitation and activation of the muscle
    /// \param emgMax Value of the maximal excitation and activation of the muscle
    ///
    void setStateMax(
        const State &emgMax);

    ///
    /// \brief Return the maximal excitation and activation of the muscle
    /// \return The maximal excitation and activation of the muscle
    ///
    const State& stateMax() const;

    ///
    /// \brief Set the fatigue parameters
    /// \param fatigueParameters The values of the fatigue parameters
    ///
    void setFatigueParameters(
        const FatigueParameters& fatigueParameters);
    ///
    /// \brief Return the fatigue parameters
    /// \return The fatigue parameters
    ///
    const FatigueParameters& fatigueParameters() const;
    
    ///
    /// \brief Choose if use damping for muscle force computation
    /// \param val 0 to not use damping 
    ///
    void setUseDamping(
        bool val);

    ///
    /// \brief Return 1 if use damping for muscle computation
    /// \return 0 if not use damping 1 overall
    ///
    bool useDamping() const;


protected:
    std::shared_ptr<utils::Scalar> m_optimalLength; ///< Length without tension
    std::shared_ptr<utils::Scalar> m_fIsoMax;       ///< Maximal isometric force
    std::shared_ptr<utils::Scalar> m_PCSA;          ///< Physiological cross-sectional area of the muscle
    std::shared_ptr<utils::Scalar> m_tendonSlackLength; ///< Tendon slack length
    std::shared_ptr<utils::Scalar> m_pennationAngle; ///< Angle of pennation
    std::shared_ptr<bool> m_useDamping;             ///< Use damping for muscle force from activation
    std::shared_ptr<State> m_stateMax;              ///< Maximal excitation et activation of the muscle

    std::shared_ptr<utils::Scalar> m_maxShorteningSpeed; ///< Maximal velocity of shortening

    // Activation parameters
    std::shared_ptr<utils::Scalar> m_minActivation; ///< Minimal activation
    std::shared_ptr<utils::Scalar> m_torqueActivation; ///<  Time activation constant
    std::shared_ptr<utils::Scalar> m_torqueDeactivation; ///< Time deactivation constant

    // Fatigue parameters
    std::shared_ptr<FatigueParameters> m_fatigueParameters; ///< Fatigue parameters
};

}
}
}

#endif // BIORBD_MUSCLES_CHARACTERISTICS_H
