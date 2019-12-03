#ifndef BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H
#define BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H

#include "biorbdConfig.h"
#include "Muscles/FatigueDynamicState.h"

namespace biorbd {
namespace muscles {

///
/// \brief Class FatigueDynamicStateXia that holds FatigueDynamicState class
/// 
class BIORBD_API FatigueDynamicStateXia : public biorbd::muscles::FatigueDynamicState
{
public:
    ///
    /// \brief Construct FatigueDynamicStateXia
    /// \param active Active muscle (default:1)
    /// \param fatigued Fatigued muscle (default: 0)
    /// \param resting Resting muscle (default: 0)
    ///
    FatigueDynamicStateXia(
            double active = 1,
            double fatigued = 0,
            double resting = 0);

    ///
    /// \brief Contruct FatigueDynamicsStateXia from another fatigue state
    /// \param m The other fatigue state
    ///
    FatigueDynamicStateXia(const std::shared_ptr<biorbd::muscles::FatigueState> m);

    ///
    /// \brief Deep copy of the fatigue dynamic state xia
    /// \return Deep copy of fatigue dynamic state xia
    ///
    biorbd::muscles::FatigueDynamicStateXia DeepCopy() const;

    ///
    /// \brief Deep copy of a fatigue dynamic state xia
    /// \param other The fatigue dynamic state xia to copy
    ///
    void DeepCopy(const biorbd::muscles::FatigueDynamicStateXia& other);

    ///
    /// \brief Time derivative state TODO?
    /// \param emg The EMG data
    /// \param characteristics The muscle characteristics
    ///
    virtual void timeDerivativeState(
            const biorbd::muscles::StateDynamics &emg,
            const biorbd::muscles::Characteristics &characteristics
     );
protected:
    ///
    /// \brief Set type of fatigue dynamic state xia
    ///
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H
