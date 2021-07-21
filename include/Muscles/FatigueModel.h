#ifndef BIORBD_MUSCLES_FATIGUE_MODEL_H
#define BIORBD_MUSCLES_FATIGUE_MODEL_H

#include <memory>
#include "biorbdConfig.h"
#include "MusclesEnums.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace BIORBD_MATH_NAMESPACE
{
namespace utils
{
class String;
}

namespace muscles
{
class Muscle;
class StateDynamics;
class FatigueState;

///
/// \brief Class Fatigue model of a muscle
///
class BIORBD_API FatigueModel
{
public:
    ///
    /// \brief Contruct a fatigable muscle model
    /// \param dynamicFatigueType The type of fatigue of the muscle
    ///
    FatigueModel(
        STATE_FATIGUE_TYPE dynamicFatigueType);

    ///
    /// \brief Construct a fatigable muscle model from another muscle
    /// \param other The other fatigue model
    ///
    FatigueModel(
        const FatigueModel& other);

    ///
    /// \brief Construct a fatigable muscle model from another muscle
    /// \param other The other fatigue model
    ///
    FatigueModel(
        const std::shared_ptr<FatigueModel> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~FatigueModel() = 0;

    ///
    /// \brief Deep copy of the fatigue model
    /// \param other The other fatigue model to copy
    ///
    void DeepCopy(
        const FatigueModel& other);

    ///
    /// \brief Compute the time derivative state
    /// \param emg EMG data
    ///
    virtual void computeTimeDerivativeState(
        const StateDynamics& emg);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Set the fatigue state
    /// \param active
    /// \param fatigued
    /// \param resting
    virtual void setFatigueState(
        const utils::Scalar& active,
        const utils::Scalar& fatigued,
        const utils::Scalar& resting);
#endif

    ///
    /// \brief Return the fatigue state
    /// \return The fatigue state
    ///
    FatigueState& fatigueState();
    ///
    /// \brief Return the fatigue state
    /// \return The fatigue state
    ///
    const FatigueState& fatigueState() const;


protected:
    std::shared_ptr<FatigueState>
    m_fatigueState; ///< The fatigue state

};

}
}
}

#endif // BIORBD_MUSCLES_FATIGUE_MODEL_H
