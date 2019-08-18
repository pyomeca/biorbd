#ifndef BIORBD_MUSCLES_MUSCLE_GROUP_H
#define BIORBD_MUSCLES_MUSCLE_GROUP_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Muscles/PathChangers.h"

namespace biorbd {
namespace muscles {

class Muscle;
class Geometry;
class Caracteristics;
class BIORBD_API MuscleGroup
{
public:
    MuscleGroup(
            const biorbd::utils::String &name,
            const biorbd::utils::String &originName,
            const biorbd::utils::String &insertionName);
    virtual ~MuscleGroup();

    virtual void addHillMuscle(
            const biorbd::utils::String&,
            const biorbd::utils::String&,
            const biorbd::muscles::Geometry&,
            const biorbd::muscles::Caracteristics&,
            const biorbd::muscles::PathChangers& = biorbd::muscles::PathChangers(),
            const biorbd::utils::String& stateType = "default",
            const biorbd::utils::String &dynamicFatigueType = "Simple");
    virtual void addMuscle(biorbd::muscles::Muscle &val);

    // Set and get
    unsigned int nbMuscles() const;
    std::shared_ptr<biorbd::muscles::Muscle> muscle_nonConst(const unsigned int &idx);
    const std::shared_ptr<biorbd::muscles::Muscle> muscle(const unsigned int &idx) const;
    int muscleID(const biorbd::utils::String&); // Retourne l'index du muscle
    void setName(const biorbd::utils::String& name);
    void setOrigin(const biorbd::utils::String& name);
    void setInsertion(const biorbd::utils::String& name);
    const biorbd::utils::String& name() const;
    const biorbd::utils::String& origin() const;
    const biorbd::utils::String& insertion() const;

protected:
    std::vector<std::shared_ptr<biorbd::muscles::Muscle>> m_mus;
    biorbd::utils::String m_name;
    biorbd::utils::String m_originName;
    biorbd::utils::String m_insertName;

};

}}

#endif // BIORBD_MUSCLES_MUSCLE_GROUP_H
