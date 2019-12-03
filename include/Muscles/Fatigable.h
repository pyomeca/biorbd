#ifndef BIORBD_MUSCLES_FATIGABLE_H
#define BIORBD_MUSCLES_FATIGABLE_H

#include <memory>
#include "biorbdConfig.h"
#include "MusclesEnums.h"

namespace biorbd {
namespace utils {
class String;
}

namespace muscles {
class Muscle;
class StateDynamics;
class FatigueState;

///
/// \brief Class fatigable 
///
class BIORBD_API Fatigable
{
public:
    ///
    /// \brief Contruct fatigable 
    /// \param dynamicFatigueType The type of fatigue of the muscle
    ///
    Fatigable(biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType);

    ///
    /// \brief Construct fatigable from other fatigable
    /// \param m Other fatigable
    ///
    Fatigable(const biorbd::muscles::Fatigable& m);

    ///
    /// \brief Construct fatigable from other fatigable
    /// \param m Other fatigable (pointer)
    ///
    Fatigable(const std::shared_ptr<biorbd::muscles::Fatigable> m);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Fatigable() = 0;

    /// 
    /// \brief Deep copy of fatigable 
    /// \param other The fatigable to copy 
    ///
    void DeepCopy(const biorbd::muscles::Fatigable& other);

    ///
    /// \brief Compute the time derivative state
    /// \param emg EMG data
    ///
    virtual void computeTimeDerivativeState(const biorbd::muscles::StateDynamics& emg);

    ///
    /// \brief Return the fatigue state
    /// \return The fatigue state
    ///
    biorbd::muscles::FatigueState& fatigueState();
    ///
    /// \brief Return the fatigue state
    /// \return The fatigue state
    ///
    const biorbd::muscles::FatigueState& fatigueState() const;

    ///
    /// \brief Set the fatigue state
    /// \param active
    /// \param fatigued
    /// \param resting
    virtual void fatigueState(double active, double fatigued, double resting);

protected:
    std::shared_ptr<biorbd::muscles::FatigueState> m_fatigueState; ///< The fatigue state

};

}}

#endif // BIORBD_MUSCLES_FATIGABLE_H
