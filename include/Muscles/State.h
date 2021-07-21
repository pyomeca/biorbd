#ifndef BIORBD_MUSCLES_STATE_H
#define BIORBD_MUSCLES_STATE_H

#include <memory>
#include "biorbdConfig.h"
#include "Muscles/MusclesEnums.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace muscles
{
///
/// \brief EMG holder to interact with the muscle
///
class BIORBD_API State
{
public:
    ///
    /// \brief Construct a state
    /// \param excitation The muscle excitation
    /// \param activation The muscle activation
    ///
    State(
        const utils::Scalar& excitation = 0,
        const utils::Scalar& activation = 0);

    ///
    /// \brief Construct a muscle state from another state
    /// \param other The other state
    ///
    State(
        const State& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~State();

    ///
    /// \brief Deep copy of state
    /// \return A deep copy of state
    ///
    State DeepCopy() const;

    ///
    /// \brief Deep copy of state into another state
    /// \param other The state to copy
    ///
    void DeepCopy(
        const State& other);

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
    /// \brief Return the muscle excitation
    /// \return The muscle excitation
    ///
    const utils::Scalar& excitation() const;

    ///
    /// \brief Compute and return the normalized excitation
    /// \param emgMax The maximal emg
    /// \param turnOffWarnings If the warning when excitation is higher than 1 should be off or on
    /// \return The normalized excitation
    ///
    /// Even when the warning is ON, the computation is performed anyway
    ///
    const utils::Scalar& normalizeExcitation(
        const State &emgMax,
        bool turnOffWarnings = false);

    ///
    /// \brief Force set the normalized excitation
    /// \param val Value of the normalized excitation to set
    ///
    void setExcitationNorm(
        const utils::Scalar& val);

    ///
    /// \brief Return the previously normalized excitation
    /// \return The normalized excitation
    ///
    const utils::Scalar& excitationNorm() const;

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
    /// \brief Return the muscle activation
    /// \return The muscle activation
    ///
    const utils::Scalar& activation() const;

    ///
    /// \brief Return the state type
    /// \return The state type
    ///
    STATE_TYPE type() const;
protected:
    ///
    /// \brief Set the type to simple_state
    ///
    virtual void setType();

    std::shared_ptr<STATE_TYPE> m_stateType;///< The state type
    std::shared_ptr<utils::Scalar> m_excitation;///< The muscle excitation
    std::shared_ptr<utils::Scalar>
    m_excitationNorm; ///< The normalized excitation
    std::shared_ptr<utils::Scalar> m_activation;///< The muscle activation

};

}
}

#endif // BIORBD_MUSCLES_STATE_H
