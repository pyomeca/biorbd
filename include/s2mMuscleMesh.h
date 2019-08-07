#ifndef S2MMUSCLEMESH_H
#define S2MMUSCLEMESH_H

#include "biorbdConfig.h"
#include "s2mMuscleCompound.h"

class s2mMuscle;
class BIORBD_API s2mMuscleMesh : public s2mMuscleCompound
{
    public:
        s2mMuscleMesh();
        ~s2mMuscleMesh();

        // Surcharge d'opérateur
        virtual s2mMuscleMesh& operator= (const s2mMuscleMesh &o);


        // Set and get
        const s2mMuscle& getMuscleLine(const unsigned int &idx) const; // Get sur une ligne d'action
        virtual void addLineOfAction(s2mMuscle&); // Ajouter une ligne d'action
    protected:
        // Fonction virtuelle pure
        virtual s2mMuscleForce computeForce() = 0;

        s2mMuscle ** m_muscles;
        unsigned int m_nbMuscles;
        virtual void assignValue(s2mMuscle*&, s2mMuscle*&);
};

#endif // S2MMUSCLEMESH_H
