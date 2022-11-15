#ifndef BIORBD_MUSCLES_MUSCLE_GROUP_H
#define BIORBD_MUSCLES_MUSCLE_GROUP_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"
#include "InternalForces/Muscles/MusclesEnums.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
}

namespace internal_forces
{
class Geometry;
class PathModifiers;

namespace muscles
{
class Muscle;
class Characteristics;

///
/// \brief A muscle group is muscle that share parents for both origin and insertion
///
class BIORBD_API MuscleGroup
{
public:
    ///
    /// \brief Construct a muscle group
    ///
    MuscleGroup();

    ///
    /// \brief Construct a muscle group from another muscle group
    /// \param other The other muscle group
    ///
    MuscleGroup(
        const MuscleGroup& other);

    ///
    /// \brief Construct a muscle group
    /// \param name The name of the muscle group
    /// \param originName The segment name where the origin lies
    /// \param insertionName The segment name where the insertion lies
    ///
    MuscleGroup(
        const utils::String &name,
        const utils::String &originName,
        const utils::String &insertionName);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~MuscleGroup();

    ///
    /// \brief Deep copy of a muscle group
    /// \return A deep copy of a muscle group
    ///
    MuscleGroup DeepCopy() const;

    ///
    /// \brief Deep copy of a muscle group in new muscle group
    /// \param other The muscle group to copy
    ///
    void DeepCopy(
        const MuscleGroup& other);

#ifndef SWIG
    ///
    /// \brief To add a muscle to the group
    /// \param name The name of the muscle
    /// \param type The muscle type
    /// \param geometry The geometry of the muscle
    /// \param characteristics The muscle characteristics
    /// \param stateType The state stype
    /// \param dynamicFatigueType The dynamic state fatigue type
    ///
    virtual void addMuscle(
        const utils::String& name,
        MUSCLE_TYPE type,
        const internal_forces::Geometry& geometry,
        const Characteristics& characteristics,
        STATE_TYPE stateType =
            STATE_TYPE::NO_STATE_TYPE,
        STATE_FATIGUE_TYPE dynamicFatigueType =
            STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

    ///
    /// \brief To add a muscle to the group
    /// \param name The name of the muscle
    /// \param type The muscle type
    /// \param geometry The geometry of the muscle
    /// \param characteristics The muscle characteristics
    /// \param dynamicFatigueType The dynamic state fatigue type
    ///
    virtual void addMuscle(
        const utils::String& name,
        MUSCLE_TYPE type,
        const internal_forces::Geometry& geometry,
        const Characteristics& characteristics,
        STATE_FATIGUE_TYPE dynamicFatigueType);

    ///
    /// \brief To add a muscle to the group
    /// \param name The name of the muscle
    /// \param type The muscle type
    /// \param geometry The geometry of the muscle
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param stateType The state stype
    /// \param dynamicFatigueType The dynamic state fatigue type
    ///
    virtual void addMuscle(
        const utils::String& name,
        MUSCLE_TYPE type,
        const internal_forces::Geometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers,
        STATE_TYPE stateType =
            STATE_TYPE::NO_STATE_TYPE,
        STATE_FATIGUE_TYPE dynamicFatigueType =
            STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

    ///
    /// \brief To add a muscle to the group
    /// \param name Name of the muscle
    /// \param type The muscle type
    /// \param geometry The geometry of the muscle
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param dynamicFatigueType The dynamic state fatigue type
    ///
    virtual void addMuscle(
        const utils::String& name,
        MUSCLE_TYPE type,
        const internal_forces::Geometry& geometry,
        const Characteristics& characteristics,
        const internal_forces::PathModifiers& pathModifiers,
        STATE_FATIGUE_TYPE dynamicFatigueType);
#endif

    ///
    /// \brief To add a muscle to the group
    /// \param muscle The muscle to add
    ///
    virtual void addMuscle(
        const Muscle &muscle);

    ///
    /// \brief Return the number of muscles in the group
    /// \return The number of muscles in the group
    ///
    unsigned int nbMuscles() const;

    ///
    /// \brief Return the muscles in the group
    /// \return The muscles
    ///
    std::vector<std::shared_ptr<Muscle>>& muscles();

    ///
    /// \brief Return the muscles in the group
    /// \return The muscles
    ///
    const std::vector<std::shared_ptr<Muscle>>& muscles() const;

    ///
    /// \brief Return the muscle of a specific index in the group
    /// \param idx The muscle index
    /// \return The muscle of a specific index
    ///
    Muscle& muscle(
        unsigned int idx);

    ///
    /// \brief Return the muscle of a specific index in the group
    /// \param idx The muscle index
    /// \return The muscle of a specific index
    ///
    const Muscle& muscle(
        unsigned int idx) const;

    ///
    /// \brief Return the muscle index
    /// \param name The name of the muscle
    /// \return The muscle index
    ///
    int muscleID(
        const utils::String& name);

    ///
    /// \brief Set the name of the muscle group
    /// \param name The name of the muscle group
    ///
    void setName(
        const utils::String& name);

    ///
    /// \brief Return the name of the muscle group
    /// \return The name of the muscle
    ///
    const utils::String& name() const;

    ///
    /// \brief Set the origin segment name where the origin lies
    /// \param name The origin segment name where the origin lies
    ///
    void setOrigin(
        const utils::String& name);

    ///
    /// \brief Return the origin segment name
    /// \return The origin segment name
    ///
    const utils::String& origin() const;

    ///
    /// \brief Set the insertion segment name where the origin lies
    /// \param name The insertion segment name where the origin lies
    ///
    void setInsertion(
        const utils::String& name);

    ///
    /// \brief Return the insertion segment name
    /// \return The insertion segment name
    ///
    const utils::String& insertion() const;

protected:
    std::shared_ptr<std::vector<std::shared_ptr<Muscle>>>
    m_mus; ///< The set of muscles
    std::shared_ptr<utils::String> m_name; ///< The muscle group name
    std::shared_ptr<utils::String> m_originName; ///<The origin name
    std::shared_ptr<utils::String> m_insertName; ///< The insertion name

};

}
}
}

#endif // BIORBD_MUSCLES_MUSCLE_GROUP_H
