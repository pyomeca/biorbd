#ifndef S2MMUSCLEMESH_H
#define S2MMUSCLEMESH_H

#include "s2mError.h"
#include "s2mMuscleForce.h"
#include "s2mMuscleCompound.h"
#include "s2mMuscleHillType.h"


class s2mMuscleMesh : public s2mMuscleCompound
{
    public:
        s2mMuscleMesh();
        ~s2mMuscleMesh();

        // Surcharge d'opérateur
        virtual s2mMuscleMesh& operator= (const s2mMuscleMesh &o);


        // Set and get
        virtual s2mMuscle& getMuscleLine(const unsigned int &idx); // Get sur une ligne d'action
        virtual void addLineOfAction(s2mMuscle&); // Ajouter une ligne d'action
    protected:
        // Fonction virtuelle pure
        virtual s2mMuscleForce computeForce() = 0;

        s2mMuscle ** m_muscles;
        unsigned int m_nbMuscles;
        virtual void assignValue(s2mMuscle*&, s2mMuscle*&);
    private:

};

#endif // S2MMUSCLEMESH_H
