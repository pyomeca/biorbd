#ifndef S2MMUSCLEMESHHILLTYPE_H
#define S2MMUSCLEMESHHILLTYPE_H

    #include "biorbdConfig.h"
    #include "s2mMuscleMesh.h"
    #include "s2mMuscleHillType.h"
    #include "s2mGenCoord.h"

class BIORBD_API s2mMuscleMeshTransverse : public s2mMuscleMesh
{
    public:
        s2mMuscleMeshTransverse();
        ~s2mMuscleMeshTransverse();

        // Muscles components
        virtual s2mMuscleStateActual dynamicMuscleActivation() ;
        virtual s2mMuscleStateActual dynamicMuscleContraction() ;

        // Get sur la force
        virtual std::vector<std::shared_ptr<s2mMuscleForce> > force(s2mJoints& , const s2mGenCoord&, const s2mMuscleStateActual&) {std::vector<std::shared_ptr<s2mMuscleForce> > dummy; return dummy;}
        virtual std::vector<std::shared_ptr<s2mMuscleForce> > force(const s2mMuscleStateActual&) {std::vector<std::shared_ptr<s2mMuscleForce> > dummy; return dummy;}

        // New fmethods
        virtual void addTransverseComponent() {}

        // Set and Get
        virtual void setNbTransverseElement(const unsigned int);
    protected:
        virtual void setType() {m_type = "Transverse";}
        virtual void setForce() {}
        //Résolution des fonctions virtuelles pures
        // Muscles components
        virtual s2mMuscleStateActual computeDynamicMuscleActivation();
        virtual s2mMuscleStateActual computeDynamicMuscleContraction();
        virtual s2mMuscleGeometry updateMuscleGeometry();
        // Muscles mesh
        virtual s2mMuscleForce computeForce();

        unsigned int m_nbTrans;
    private:
};

#endif // S2MMUSCLEMESHHILLTYPE_H
