#ifndef S2MMUSCLEPATHCHANGERS_H
#define S2MMUSCLEPATHCHANGERS_H
    #include <memory>
    #include <vector>
    #include "biorbdConfig.h"
    #include "s2mString.h"
    #include "s2mMusclePathChanger.h"
    #include "s2mViaPoint.h"
    #include "s2mWrappingSphere.h"
    #include "s2mWrappingCylinder.h"




class BIORBD_API s2mMusclePathChangers
{
    public:
        s2mMusclePathChangers();
        ~s2mMusclePathChangers();
        virtual void addPathChanger(s2mMusclePathChanger&); // Ajouter un wrapping ou un via point

        // Set and get
        virtual unsigned int nbWraps() const { return m_nbWraps; } // retourne le nombre total de wrapping objects
        virtual unsigned int nbVia() const { return m_nbVia; } // retourne le nombre total de via points
        virtual unsigned int nbObjects() const {return m_totalObjects; } // Retourne le nombre total d'objects
        virtual std::shared_ptr<s2mMusclePathChanger> object(const unsigned int&) const; // Get sur un wrapping

    protected:
        std::vector<std::shared_ptr<s2mMusclePathChanger> > m_obj; // Tableau de pointeur sur les objects
        unsigned int m_nbWraps;
        unsigned int m_nbVia;
        unsigned int m_totalObjects;

    private:
};

#endif // S2MMUSCLEPATHCHANGERS_H
