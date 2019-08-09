#ifndef S2MMUSCLEPATHCHANGERS_H
#define S2MMUSCLEPATHCHANGERS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

class s2mMusclePathChanger;
class BIORBD_API s2mMusclePathChangers
{
    public:
        s2mMusclePathChangers();
        virtual ~s2mMusclePathChangers();
        void addPathChanger(s2mMusclePathChanger&); // Ajouter un wrapping ou un via point

        // Set and get
        unsigned int nbWraps() const; // retourne le nombre total de wrapping objects
        unsigned int nbVia() const; // retourne le nombre total de via points
        unsigned int nbObjects() const; // Retourne le nombre total d'objects
        const std::shared_ptr<s2mMusclePathChanger> object(const unsigned int& idx) const; // Get sur un wrapping

    protected:
        std::vector<std::shared_ptr<s2mMusclePathChanger>> m_obj; // Tableau de pointeur sur les objects
        unsigned int m_nbWraps;
        unsigned int m_nbVia;
        unsigned int m_totalObjects;

};

#endif // S2MMUSCLEPATHCHANGERS_H
