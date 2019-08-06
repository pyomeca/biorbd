#ifndef S2MGROUPEMUSCULAIRE_H
#define S2MGROUPEMUSCULAIRE_H
    #include "biorbdConfig.h"
    #include "s2mError.h"
    #include <memory>
    #include <vector>
    #include "s2mMuscleStateActualBuchanan.h"
    #include "s2mMuscleMeshTransverse.h"
    #include "s2mMuscleHillType.h"
    #include "s2mMuscleHillTypeMaxime.h"
    #include "s2mMuscleHillTypeChadwick.h"
    #include "s2mMuscleHillTypeThelen.h"
    #include "s2mMuscleHillTypeSchutte.h"
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
        virtual unsigned int nbMuscles() const;
        virtual std::shared_ptr<s2mMuscle> muscle(const unsigned int &idx);
        virtual int muscleID(const s2mString&); // Retourne l'index du muscle
        virtual void setName(s2mString name);
        virtual void setOrigin(s2mString name);
        virtual void setInsertion(s2mString name);
        virtual s2mString name() const;
        virtual s2mString origin() const;
        virtual s2mString insertion() const;

    protected:
        std::vector<std::shared_ptr<s2mMuscle> > m_mus;
        s2mString m_name;
        s2mString m_originName;
        s2mString m_insertName;
    private:
};

#endif // S2MGROUPEMUSCULAIRE_H
