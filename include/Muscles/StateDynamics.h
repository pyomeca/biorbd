#ifndef BIORBD_MUSCLES_STATE_DYNAMICS_H
#define BIORBD_MUSCLES_STATE_DYNAMICS_H

#include "biorbdConfig.h"
#include "Muscles/State.h"

namespace biorbd
{
namespace BIORBD_MATH_NAMESPACE
{
namespace muscles
{
class Characteristics;
///
/// \brief EMG with the capability to compute the time derivative
///
class BIORBD_API StateDynamics : public State
{
public:
    ///
    /// \brief Construct the state dynamics
    /// \param excitation The muscle excitation
    /// \param activation The muscle activation
    ///
    StateDynamics(
        const utils::Scalar& excitation = 0,
        const utils::Scalar& activation = 0);

    ///
    /// \brief Construct a state dynamics from another state dynamics
    /// \param other The other state dynamics
    ///
    StateDynamics(
        const State& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~StateDynamics();

    ///
    /// \brief Deep copy of state dynamics
    /// \return A deep copy of state dynamics
    ///
    StateDynamics DeepCopy() const;

    ///
    /// \brief Deep copy of state dynamics into another state dynamics
    /// \param other The state dynamics to copy
    ///
    void DeepCopy(
        const StateDynamics& other);

    ///
    /// \brief Set the muscle excitation
    /// \param val The value of the muscle excitation
    /// \param turnOffWarnings If the warnings should be OFF or ON.
    ///
    /// Even if the warning on the excitation being lower than 0 is set to OFF
    /// it changes it to 0 anyway, but doesn't send a warning saying it.
    ///
    virtual void setExcitation(
        const utils::Scalar& val,
        bool turnOffWarnings = false);

    ///
    /// \brief Return the previous activation
    /// \return The previous activation
    ///
    const utils::Scalar& previousExcitation() const;

    ///
    /// \brief Set the muscle activation
    /// \param val The value of the muscle activation
    /// \param turnOffWarnings If the warnings should be OFF or ON.
    ///
    /// Even if the warning on the activation being lower than 0 is set to OFF
    /// it changes it to 0 anyway, but doesn't send a warning saying it.
    ///
    /// Even if the warning on the activation being higher than 1 is set to OFF
    /// it changes it to 1 anyway, but doesn't send a warning saying it.
    ///
    virtual void setActivation(
        const utils::Scalar& val,
        bool turnOffWarnings = false);

    ///
    /// \brief Return the previous activation
    /// \return The previous activation
    ///
    const utils::Scalar& previousActivation() const;

    ///
    /// \brief Compute and return the activation time derivative from the excitation and activation
    /// \param excitation The muscle excitation
    /// \param activation The muscle activation
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized
    /// \return The activation time derivative
    ///
    virtual const utils::Scalar& timeDerivativeActivation(
        const utils::Scalar& excitation,
        const utils::Scalar& activation,
        const Characteristics& characteristics,
        bool alreadyNormalized = false);

    ///
    /// \brief Compute and return the activation time derivative
    /// \param emg The emg
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized
    /// \return The activation time derivative
    ///
    virtual const utils::Scalar& timeDerivativeActivation(
        const State& emg,
        const Characteristics& characteristics,
        bool alreadyNormalized = false);

    ///
    /// \brief Compute and return the activation time derivative
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized
    /// \return The activation time derivative
    ///
    virtual const utils::Scalar& timeDerivativeActivation(
        const Characteristics& characteristics,
        bool alreadyNormalized = false);

    ///
    /// \brief Return the previously computed activation time derivative
    /// \return The activation time derivative
    ///
    virtual const utils::Scalar& timeDerivativeActivation();

protected:
    virtual void setType();
    std::shared_ptr<utils::Scalar>
    m_previousExcitation; ///< The previous excitation
    std::shared_ptr<utils::Scalar>
    m_previousActivation; ///<The previous activation
    std::shared_ptr<utils::Scalar>
    m_activationDot;///< The activation velocity

};

}
}
}

#endif // BIORBD_MUSCLES_STATE_DYNAMICS_H
