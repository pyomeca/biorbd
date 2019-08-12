#ifndef S2M_GROUPE_MUSCULAIRE_H
#define S2M_GROUPE_MUSCULAIRE_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "s2mMusclePathChangers.h"

class s2mMuscle;
class s2mMuscleGeometry;
class s2mMuscleCaracteristics;
class BIORBD_API s2mGroupeMusculaire
{
public:
    s2mGroupeMusculaire(
            const biorbd::utils::String &name,
            const biorbd::utils::String &originName,
            const biorbd::utils::String &insertionName);
    virtual ~s2mGroupeMusculaire();

    virtual void addHillMuscle(
            const biorbd::utils::String&,
            const biorbd::utils::String&,
            const s2mMuscleGeometry&,
            const s2mMuscleCaracteristics&,
            const s2mMusclePathChangers& = s2mMusclePathChangers(),
            const biorbd::utils::String& stateType = "default",
            const biorbd::utils::String &dynamicFatigueType = "Simple");
    virtual void addMuscle(s2mMuscle &val);

    // Set and get
    unsigned int nbMuscles() const;
    std::shared_ptr<s2mMuscle> muscle_nonConst(const unsigned int &idx);
    const std::shared_ptr<s2mMuscle> muscle(const unsigned int &idx) const;
    int muscleID(const biorbd::utils::String&); // Retourne l'index du muscle
    void setName(const biorbd::utils::String& name);
    void setOrigin(const biorbd::utils::String& name);
    void setInsertion(const biorbd::utils::String& name);
    const biorbd::utils::String& name() const;
    const biorbd::utils::String& origin() const;
    const biorbd::utils::String& insertion() const;

protected:
    std::vector<std::shared_ptr<s2mMuscle>> m_mus;
    biorbd::utils::String m_name;
    biorbd::utils::String m_originName;
    biorbd::utils::String m_insertName;

};

#endif // S2M_GROUPE_MUSCULAIRE_H
