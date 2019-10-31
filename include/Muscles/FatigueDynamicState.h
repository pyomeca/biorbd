#ifndef BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H
#define BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H

#include "biorbdConfig.h"
#include "Muscles/FatigueState.h"

namespace biorbd {
namespace muscles {
class StateDynamics;
class Characteristics;

///
/// \brief Class FatigueDynamicState that holds the FatigueState class
///
class BIORBD_API FatigueDynamicState : public biorbd::muscles::FatigueState
{
public:
    ///
    /// \brief Construct the fatigue dynamic state
    /// \param active Muscle activation (default: 0)
    /// \param fatigued Muscle fatigue (default: 0)
    /// \param resting Muscle resting (default: 1)
    ///
    FatigueDynamicState(
            double active = 0,
            double fatigued = 0,
            double resting = 1);

    ///
    /// \brief Construct the fatigue dynamic state from other state
    /// \param m The other fatigue state
    ///
    FatigueDynamicState(const std::shared_ptr<biorbd::muscles::FatigueState> m);

    ///
    /// \brief Deep copy of a fatigue dynamic state
    /// 
    void DeepCopy(const biorbd::muscles::FatigueDynamicState& other);

    ///
    /// \brief Return the active fibers velocity? TODO
    /// \return The active fibers velocity? TODO
    ///
    double activeFibersDot() const;

    ///
    /// \brief Return the fatigued fibers velocity? TODO
    /// \return The fatigued fibers velocity? TODO
    ///
    double fatiguedFibersDot() const;

    ///
    /// \brief Return the resting fibers velocity? TODO
    /// \return The resting fibers velocity? TODO
    ///
    double restingFibersDot() const;

    ///
    /// \brief TODO?
    /// \param emg EMG data
    /// \param characteristics The muscle characteristics
    ///
    virtual void timeDerivativeState(
            const biorbd::muscles::StateDynamics &emg,
            const biorbd::muscles::Characteristics &characteristics
     ) = 0;

protected:
    std::shared_ptr<double> m_activeFibersDot; ///< The active fibers velocity? TODO
    std::shared_ptr<double> m_fatiguedFibersDot; ///< The fatigued fibers velocity? TODO
    std::shared_ptr<double> m_restingFibersDot; ///< The resting fibers velocity? TODO

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H
