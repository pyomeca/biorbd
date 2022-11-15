#ifndef BIORBD_MUSCLES_HILL_DE_GROOTE_TYPE_FATIGABLE_H
#define BIORBD_MUSCLES_HILL_DE_GROOTE_TYPE_FATIGABLE_H

#include "biorbdConfig.h"

#include "InternalForces/Muscles/HillDeGrooteType.h"
#include "InternalForces/Muscles/FatigueModel.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
}

namespace internal_forces
{
class Geometry;
namespace muscles
{

///
/// \brief Add a fatigue model to the HillDeGroote type
/// Note that useful defaults values for the FatigueParameters characteristics are:
/// fatigueRate = 0.01
/// recoveryRate = 0.002
/// developFactor = 10
/// recoverFactor = 10
///
class BIORBD_API HillDeGrooteTypeFatigable : public
    HillDeGrooteType, public FatigueModel
{
public:
    ///
    /// \brief Contruct a Hill-DeGroote-type fatigable muscle
    ///
    HillDeGrooteTypeFatigable();

    ///
    /// \brief Construct a Hill-DeGroote-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param dynamicFatigueType The muscle dynamic fatigue type
    ///
    HillDeGrooteTypeFatigable(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        STATE_FATIGUE_TYPE dynamicFatigueType =
            STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-DeGroote-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param emg The muscle dynamic state
    /// \param dynamicFatigueType The muscle dynamic fatigue model
    ///
    HillDeGrooteTypeFatigable(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const State& emg,
        STATE_FATIGUE_TYPE dynamicFatigueType =
            STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-DeGroote-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param dynamicFatigueType The muscle dynamic fatigue model
    ///
    HillDeGrooteTypeFatigable(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers,
        STATE_FATIGUE_TYPE dynamicFatigueType =
            STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-DeGroote-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param emg The muscle dynamic state
    /// \param dynamicFatigueType The muscle dynamic fatigue model
    ///
    HillDeGrooteTypeFatigable(
        const utils::String& name,
        const Geometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers,
        const State& emg,
        STATE_FATIGUE_TYPE dynamicFatigueType =
            STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-DeGroote-type fatigable muscle from another muscle
    /// \param other The other muscle
    ///
    HillDeGrooteTypeFatigable(
        const Muscle& other);

    ///
    /// \brief Construct a Hill-DeGroote-type muscle from another muscle
    /// \param other The other muscle (pointer)
    ///
    HillDeGrooteTypeFatigable(
        const std::shared_ptr<Muscle> other);

    ///
    /// \brief Deep copy of a Hill-DeGroote-type fatigable muscle
    /// \return A deep copy of a Hill-DeGroote-type fatigable muscle
    ///
    HillDeGrooteTypeFatigable DeepCopy() const;


    ///
    /// \brief Deep copy of a Hill-DeGroote-type fatigable muscle in a new Hill-DeGroote-type fatigable muscle
    /// \param other The Hill-DeGroote-type fatigable muscle to copy
    ///
    void DeepCopy(const HillDeGrooteTypeFatigable& other);

    ///
    /// \brief Compute the Force-Length of the contractile element
    /// \param emg EMG data
    ///
    virtual void computeFlCE(const State &emg);

protected:
    ///
    /// \brief Set type to Hill_DeGroote_Fatigable
    ///
    virtual void setType();

};

}
}
}

#endif // BIORBD_MUSCLES_HILL_DE_GROOTE_TYPE_FATIGABLE_H
