#ifndef S2MMUSCLECOMPOUND_H
#define S2MMUSCLECOMPOUND_H
    #include "s2mString.h"
    #include "s2mMuscleForce.h"
    #include "s2mMuscleGeometry.h"
    #include "s2mMuscleCaracteristics.h"
    #include "s2mMuscleStateActual.h"
    #include "s2mMusclePathChangers.h"
    #include "s2mGenCoord.h"


class s2mMuscleCompound
{
    public:
        s2mMuscleCompound(const s2mString &name = "", const s2mMusclePathChangers& = s2mMusclePathChangers());
        ~s2mMuscleCompound();

        // Wrapping object
        virtual s2mMusclePathChangers& pathChanger();
        virtual void addPathObject(s2mMusclePathChanger &w); // Ajouter un wrapping object


        virtual s2mString type() const {return m_type;}
        virtual std::vector<boost::shared_ptr<s2mMuscleForce> > force(s2mJoints& , const s2mGenCoord&, const s2mGenCoord&, const s2mMuscleStateActual&, const int = 2) = 0;
        virtual std::vector<boost::shared_ptr<s2mMuscleForce> > force(s2mJoints& , const s2mGenCoord&, const s2mMuscleStateActual&, const int = 2) = 0;
        virtual std::vector<boost::shared_ptr<s2mMuscleForce> > force(const s2mMuscleStateActual&) = 0;
        virtual std::vector<boost::shared_ptr<s2mMuscleForce> > force(); // Return the last computed muscle force
        virtual s2mString name() const;
        virtual void setName(const s2mString& name);
    protected:
        s2mMusclePathChangers m_pathChanger;
        std::vector<boost::shared_ptr<s2mMuscleForce> > m_force;
        virtual void setForce() = 0;
        virtual void setType()=0;
        s2mString m_type;
        s2mString m_name;
    private:
};

#endif // S2MMUSCLECOMPOUND_H
