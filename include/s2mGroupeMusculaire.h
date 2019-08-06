#ifndef S2MGROUPEMUSCULAIRE_H
#define S2MGROUPEMUSCULAIRE_H
    #include "biorbdConfig.h"
    #include "s2mError.h"
    #include <memory>
    #include <vector>
    #include "s2mMuscleStateDynamicsBuchanan.h"
    #include "s2mMuscleHillType.h"
    #include "s2mMuscleHillTypeThelen.h"
    #include "s2mMuscleHillTypeSimple.h"
    #include "s2mString.h"

class BIORBD_API s2mGroupeMusculaire
{
    public:
        s2mGroupeMusculaire(const s2mString &name, const s2mString &originName, const s2mString &insertionName);
        virtual ~s2mGroupeMusculaire();

        virtual void addHillMuscle(const s2mString&,
                               const s2mString&,
                               const s2mMuscleGeometry&,
                               const s2mMuscleCaracteristics&,
                               const s2mMusclePathChangers& = s2mMusclePathChangers(),
                               const s2mString& stateType = "default");
        virtual void addMuscle(s2mMuscle &val);

        // Set and get
        unsigned int nbMuscles() const;
        std::shared_ptr<s2mMuscle> muscle_nonConst(const unsigned int &idx);
        const std::shared_ptr<s2mMuscle> muscle(const unsigned int &idx) const;
        int muscleID(const s2mString&); // Retourne l'index du muscle
        void setName(s2mString name);
        void setOrigin(s2mString name);
        void setInsertion(s2mString name);
        const s2mString& name() const;
        const s2mString& origin() const;
        const s2mString& insertion() const;

    protected:
        std::vector<std::shared_ptr<s2mMuscle>> m_mus;
        s2mString m_name;
        s2mString m_originName;
        s2mString m_insertName;
    private:
};

#endif // S2MGROUPEMUSCULAIRE_H
