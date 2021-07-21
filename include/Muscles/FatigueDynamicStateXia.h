#ifndef BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H
#define BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H

#include "biorbdConfig.h"
#include "Muscles/FatigueDynamicState.h"

namespace biorbd
{
namespace BIORBD_MATH_NAMESPACE
{
namespace muscles
{

///
/// \brief Implementation of Xia (https://www.sciencedirect.com/science/article/pii/S0021929008003692) dynamic fatigue model
///
class BIORBD_API FatigueDynamicStateXia : public
    FatigueDynamicState
{
public:
    ///
    /// \brief Construct FatigueDynamicStateXia
    /// \param active Active muscle
    /// \param fatigued Fatigued muscle
    /// \param resting Resting muscle
    ///
    FatigueDynamicStateXia(
        const utils::Scalar& active = 1,
        const utils::Scalar& fatigued = 0,
        const utils::Scalar& resting = 0);

    ///
    /// \brief Contruct FatigueDynamicsStateXia from another fatigue state
    /// \param other The other fatigue state
    ///
    FatigueDynamicStateXia(
        const std::shared_ptr<FatigueState> other);

    ///
    /// \brief Deep copy of the fatigue dynamic state xia
    /// \return Deep copy of fatigue dynamic state xia
    ///
    FatigueDynamicStateXia DeepCopy() const;

    ///
    /// \brief Deep copy of a fatigue dynamic state xia
    /// \param other The fatigue dynamic state xia to copy
    ///
    void DeepCopy(
        const FatigueDynamicStateXia& other);

    ///
    /// \brief Time derivative state of the current states
    /// \param emg The EMG data
    /// \param characteristics The muscle characteristics
    ///
    virtual void timeDerivativeState(
        const StateDynamics &emg,
        const Characteristics &characteristics
    );
protected:
    ///
    /// \brief Set type of fatigue dynamic state xia
    ///
    virtual void setType();

};

}
}
}

#endif // BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H
