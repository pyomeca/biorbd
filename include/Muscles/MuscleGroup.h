#ifndef BIORBD_MUSCLES_MUSCLE_GROUP_H
#define BIORBD_MUSCLES_MUSCLE_GROUP_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"
#include "Muscles/MusclesEnums.h"

namespace biorbd {
namespace utils {
class String;
}

namespace muscles {
class Muscle;
class Geometry;
class Characteristics;
class PathChangers;
///
/// \brief Class muscle group
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
    MuscleGroup(const biorbd::muscles::MuscleGroup& other);
    /// 
    /// \brief Construct a muscle group
    /// \param name The name
    /// \param originName The name of the origin
    /// \param insertionName The name of the insertion
    ///
    MuscleGroup(
            const biorbd::utils::String &name,
            const biorbd::utils::String &originName,
            const biorbd::utils::String &insertionName);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~MuscleGroup();
    ///
    /// \brief Deep copy of a muscle group 
    /// \return A deep copy of a muscle group
    ///
    biorbd::muscles::MuscleGroup DeepCopy() const;

    ///
    /// \brief Deep copy of a muscle group in new muscle group
    /// \param other The muscle group to copy
    ///
    void DeepCopy(const biorbd::muscles::MuscleGroup& other);

    ///
    /// \brief To add muscle
    /// \param name Name of the muscle
    /// \param type The muscle type
    /// \param geometry The geometry of the muscle
    /// \param characteristics The muscle characteristics
    /// \param stateType The state stype
    /// \param dynamicFatigueType The dynamic state fatigue type
    ///
    virtual void addMuscle(
            const biorbd::utils::String& name,
            biorbd::muscles::MUSCLE_TYPE type,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            biorbd::muscles::STATE_TYPE stateType = biorbd::muscles::STATE_TYPE::NO_STATE_TYPE,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

    ///
    /// \brief To add muscle
    /// \param name Name of the muscle
    /// \param type The muscle type
    /// \param geometry The geometry of the muscle
    /// \param characteristics The muscle characteristics
    /// \param dynamicFatigueType The dynamic state fatigue type
    ///
    virtual void addMuscle(
            const biorbd::utils::String& name,
            biorbd::muscles::MUSCLE_TYPE type,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType);

    ///
    /// \brief To add muscle
    /// \param name Name of the muscle
    /// \param type The muscle type
    /// \param geometry The geometry of the muscle
    /// \param characteristics The muscle characteristics
    /// \param pathChangers The wrap
    /// \param stateType The state stype
    /// \param dynamicFatigueType The dynamic state fatigue type
    ///
    virtual void addMuscle(
            const biorbd::utils::String& name,
            biorbd::muscles::MUSCLE_TYPE type,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            biorbd::muscles::STATE_TYPE stateType = biorbd::muscles::STATE_TYPE::NO_STATE_TYPE,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

    ///
    /// \brief To add muscle
    /// \param name Name of the muscle
    /// \param type The muscle type
    /// \param geometry The geometry of the muscle
    /// \param characteristics The muscle characteristics
    /// \param pathChangers The wrap
    /// \param dynamicFatigueType The dynamic state fatigue type
    ///
    virtual void addMuscle(
            const biorbd::utils::String& name,
            biorbd::muscles::MUSCLE_TYPE type,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType);

    ///
    /// \brief To add muscle
    /// \param val The muscle to add
    ///
    virtual void addMuscle(
            const biorbd::muscles::Muscle &val);

    ///
    /// \brief Return the number of muscles
    /// \return The number of muscles
    ///
    unsigned int nbMuscles() const;

    ///
    /// \brief Return the muscle at a specific index
    /// \param idx The muscle index
    /// \return The muscle at specific index
    ///
    biorbd::muscles::Muscle& muscle(
            unsigned int idx);

    ///
    /// \brief Return the muscle at a specific index
    /// \param idx The muscle index
    /// \return The muscle at specific index
    ///
    const biorbd::muscles::Muscle& muscle(
            unsigned int idx) const;
    ///
    /// \brief Return the muscle index 
    /// \param name The name of the muscle
    /// \return The muscle index
    ///
    int muscleID(
            const biorbd::utils::String& name); 

    ///
    /// \brief Set the name of the muscle
    /// \param name The name of the muscle
    ///
    void setName(
            const biorbd::utils::String& name);
    
    ///
    /// \brief Set the origin name
    /// \param name The name to set
    ///
    void setOrigin(
            const biorbd::utils::String& name);
    ///
    /// \brief Set the insertion name
    /// \param name The name to set
    ///
    void setInsertion(
            const biorbd::utils::String& name);

    ///
    /// \brief Return the name of the muscle
    /// \return The name of the muscle
    ///
    const biorbd::utils::String& name() const;
    ///
    /// \brief Return the origin name
    /// \return The origin name
    ///
    const biorbd::utils::String& origin() const;
    ///
    /// \brief Return the insertion name
    /// \return The insertion name
    ///
    const biorbd::utils::String& insertion() const;

protected:
    std::shared_ptr<std::vector<std::shared_ptr<biorbd::muscles::Muscle>>> m_mus; ///< The muscle group
    std::shared_ptr<biorbd::utils::String> m_name; ///< The muscle name
    std::shared_ptr<biorbd::utils::String> m_originName; ///<The origin name
    std::shared_ptr<biorbd::utils::String> m_insertName; ///< The insertion name

};

}}

#endif // BIORBD_MUSCLES_MUSCLE_GROUP_H
