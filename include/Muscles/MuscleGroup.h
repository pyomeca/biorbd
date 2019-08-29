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
class Caracteristics;
class PathChangers;

class BIORBD_API MuscleGroup
{
public:
    MuscleGroup(
            const biorbd::utils::String &name,
            const biorbd::utils::String &originName,
            const biorbd::utils::String &insertionName);
    virtual ~MuscleGroup();

    virtual void addMuscle(
            const biorbd::utils::String& name,
            biorbd::muscles::MUSCLE_TYPE type,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caracteristics,
            biorbd::muscles::STATE_TYPE stateType = biorbd::muscles::STATE_TYPE::NO_STATE_TYPE,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);
    virtual void addMuscle(
            const biorbd::utils::String& name,
            biorbd::muscles::MUSCLE_TYPE type,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caracteristics,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType);
    virtual void addMuscle(
            const biorbd::utils::String& name,
            biorbd::muscles::MUSCLE_TYPE type,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caracteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            biorbd::muscles::STATE_TYPE stateType = biorbd::muscles::STATE_TYPE::NO_STATE_TYPE,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);
    virtual void addMuscle(
            const biorbd::utils::String& name,
            biorbd::muscles::MUSCLE_TYPE type,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caracteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType);
    virtual void addMuscle(
            const biorbd::muscles::Muscle &val);

    // Set and get
    unsigned int nbMuscles() const;
    biorbd::muscles::Muscle& muscle(
            unsigned int idx);
    const biorbd::muscles::Muscle& muscle(
            unsigned int idx) const;
    int muscleID(
            const biorbd::utils::String& name); // Retourne l'index du muscle
    void setName(
            const biorbd::utils::String& name);
    void setOrigin(
            const biorbd::utils::String& name);
    void setInsertion(
            const biorbd::utils::String& name);
    const biorbd::utils::String& name() const;
    const biorbd::utils::String& origin() const;
    const biorbd::utils::String& insertion() const;

protected:
    std::shared_ptr<std::vector<std::shared_ptr<biorbd::muscles::Muscle>>> m_mus;
    std::shared_ptr<biorbd::utils::String> m_name;
    std::shared_ptr<biorbd::utils::String> m_originName;
    std::shared_ptr<biorbd::utils::String> m_insertName;

};

}}

#endif // BIORBD_MUSCLES_MUSCLE_GROUP_H
