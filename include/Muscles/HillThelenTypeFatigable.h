#ifndef BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H
#define BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H

#include "biorbdConfig.h"

#include "Muscles/HillThelenType.h"
#include "Muscles/Fatigable.h"

namespace biorbd {
namespace utils {
class String;
}

namespace muscles {

///
/// \brief The HillThelenTypeFatigable class
/// Class of a Thelen fatigable type.
/// Note that useful defaults values for the FatigueParameters characteristics are:
/// fatigueRate = 0.01
/// recoveryRate = 0.002
/// developFactor = 10
/// recoverFactor = 10
///
class BIORBD_API HillThelenTypeFatigable : public biorbd::muscles::HillThelenType, public biorbd::muscles::Fatigable
{
public:
    ///
    /// \brief Contruct a Hill-Thelen-type fatigable muscle
    ///
    HillThelenTypeFatigable();

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param dynamicFatigueType The muscle dynamic fatigue type
    ///
    HillThelenTypeFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param dynamicState The muscle dynamic state
    /// \param dynamicFatigueType The muscle dynamic fatigue type
    ///
    HillThelenTypeFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::StateDynamics& dynamicState,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathChangers The path changers
    /// \param dynamicFatigueType The muscle dynamic fatigue type
    ///
    HillThelenTypeFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathChangers The path changers
    /// \param dynamicState The muscle dynamic state
    /// \param dynamicFatigueType The muscle dynamic fatigue type
    ///
    HillThelenTypeFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle from another muscle
    /// \param muscle The other muscle
    ///
    HillThelenTypeFatigable(
            const biorbd::muscles::Muscle& muscle);

    ///
    /// \brief Construct a Hill-Thelen-type muscle from another muscle
    /// \param muscle The other muscle (pointer)
    ///
    HillThelenTypeFatigable(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);

    ///
    /// \brief Deep copy of a Hill-Thelen-type fatigable muscle
    /// \return A deep copy of a Hill-Thelen-type fatigable muscle
    ///
    biorbd::muscles::HillThelenTypeFatigable DeepCopy() const;


    ///
    /// \brief Deep copy of a Hill-Thelen-type fatigable muscle in a new Hill-Thelen-type fatigable muscle
    /// \param other The Hill-Thelen-type fatigable muscle to copy
    ///
    void DeepCopy(const biorbd::muscles::HillThelenTypeFatigable& other);
    ///
    /// \brief Compute the Force-Length contractile element
    /// \param EMG EMG data
    ///
    virtual void computeFlCE(const biorbd::muscles::StateDynamics &EMG);

protected:
    ///
    /// \brief Set type to Hill_Thelen_Fatigable
    ///
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H
