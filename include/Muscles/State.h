#ifndef BIORBD_MUSCLES_STATE_H
#define BIORBD_MUSCLES_STATE_H

#include <memory>
#include "biorbdConfig.h"
#include "Muscles/MusclesEnums.h"

namespace biorbd {
namespace muscles {
    ///
    /// \brief Class State
    ///
class BIORBD_API State
{
public:
    ///
    /// \brief Construct state
    /// \param excitation The muscle excitation (default: 0)
    /// \param activation The muscle activation (default: 0)
    ///
    State(
            double excitation = 0,
            double activation = 0);
    ///
    /// \brief Construct a muscle state from another state
    /// \param other The other state
    ///
    State(const biorbd::muscles::State& other);
    ///
    /// \brief Destroy class properly
    ///
    virtual ~State();
    ///
    /// \brief Deep copy of state
    /// \return A deep copy of state
    ///
    biorbd::muscles::State DeepCopy() const;
    ///
    /// \brief Deep copy of state into another state
    /// \parma other The state to copy
    ///
    void DeepCopy(const biorbd::muscles::State& other);

    ///
    /// \brief Set the muscle excitation
    /// \param val The value of the muscle excitation
    ///
    virtual void setExcitation(double val);

    ///
    /// \brief Set the muscle activation
    /// \param val The value of the muscle activation
    ///
    virtual void setActivation(double val);

    ///
    /// \brief Return the muscle excitation
    /// \return The muscle excitation
    ///
    double excitation() const;
    ///
    /// \brief Return the muscle activation
    /// \return The muscle activation
    ///
    double activation() const;
    ///
    /// \brief Return the state type
    /// \return The state type
    ///
    biorbd::muscles::STATE_TYPE type() const;
protected:
    ///
    /// \brief Set the type to simple_state
    ///
    virtual void setType();

    std::shared_ptr<biorbd::muscles::STATE_TYPE> m_stateType;///< The state type
    std::shared_ptr<double> m_excitation;///< The muscle excitation
    std::shared_ptr<double> m_activation;///< The muscle activation

};

}}

#endif // BIORBD_MUSCLES_STATE_H
